//! Files and pictures over an Ion link, in the chat's byte stream.
//!
//! A file goes as a header line and its bytes:
//!
//! ```text
//! 0x1E "HF1 " kind " " size " " name "\n" <size bytes>
//! ```
//!
//! `kind` is `file` (any bytes, saved as they are) or `img` (a picture
//! made to arrive in strips: [`image_payload`]). 0x1E (ASCII record
//! separator) does not occur in typed text, so [`Demux`] can tell the two
//! apart in one stream; text around a file stays text. A Winlink client on
//! the host interface has the stream to itself and never sees this.

use std::io::Cursor;

/// Start of a file's header line.
pub const MAGIC: &[u8] = b"\x1EHF1 ";
/// Longest header line accepted (the name is cut to fit).
const MAX_HEADER: usize = 300;
/// Largest file the receiver accepts.
pub const MAX_SIZE: usize = 16 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    File,
    Image,
}

impl Kind {
    fn word(self) -> &'static str {
        match self {
            Kind::File => "file",
            Kind::Image => "img",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub kind: Kind,
    pub size: usize,
    pub name: String,
}

/// The bytes to send for `data` named `name`.
pub fn frame(kind: Kind, name: &str, data: &[u8]) -> Vec<u8> {
    let name: String = name.chars().filter(|c| !c.is_control()).take(120).collect();
    let mut out = MAGIC.to_vec();
    out.extend(format!("{} {} {}\n", kind.word(), data.len(), name).into_bytes());
    out.extend_from_slice(data);
    out
}

/// What the stream held, in order.
#[derive(Clone, Debug, PartialEq)]
pub enum Item {
    Text(String),
    /// A file has begun: its header.
    Start(Header),
    /// Bytes of the file in progress so far (all of them, not just new).
    Progress(usize),
    /// The file is complete.
    Done(Header, Vec<u8>),
}

/// Splits the received stream into text and files.
#[derive(Default)]
pub struct Demux {
    /// Bytes not yet sorted: a partial header, or text that might start one.
    pending: Vec<u8>,
    /// The file being received and its bytes so far.
    file: Option<(Header, Vec<u8>)>,
}

impl Demux {
    pub fn new() -> Self {
        Self::default()
    }

    /// The file in progress, if any: its header and bytes so far.
    pub fn current(&self) -> Option<(&Header, &[u8])> {
        self.file.as_ref().map(|(h, b)| (h, b.as_slice()))
    }

    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Item> {
        self.pending.extend_from_slice(bytes);
        let mut out = Vec::new();
        loop {
            if let Some((h, data)) = &mut self.file {
                let take = (h.size - data.len()).min(self.pending.len());
                data.extend(self.pending.drain(..take));
                if data.len() == h.size {
                    let (h, data) = self.file.take().unwrap();
                    out.push(Item::Done(h, data));
                    continue;
                }
                if take > 0 {
                    out.push(Item::Progress(data.len()));
                }
                return out;
            }
            // Text up to a possible header.
            match self.pending.iter().position(|&b| b == MAGIC[0]) {
                None => {
                    let text = std::mem::take(&mut self.pending);
                    push_text(&mut out, &text);
                    return out;
                }
                Some(i) => {
                    if i > 0 {
                        let text: Vec<u8> = self.pending.drain(..i).collect();
                        push_text(&mut out, &text);
                    }
                }
            }
            // `pending` starts with 0x1E: a header, part of one, or not one.
            let n = self.pending.len().min(MAGIC.len());
            if self.pending[..n] != MAGIC[..n] {
                // Not ours: the separator is text.
                let text: Vec<u8> = self.pending.drain(..1).collect();
                push_text(&mut out, &text);
                continue;
            }
            let Some(end) = self.pending.iter().position(|&b| b == b'\n') else {
                if self.pending.len() > MAX_HEADER {
                    let text: Vec<u8> = self.pending.drain(..1).collect();
                    push_text(&mut out, &text);
                    continue;
                }
                return out; // wait for the rest of the header
            };
            let line: Vec<u8> = self.pending.drain(..=end).collect();
            match parse_header(&line[MAGIC.len()..line.len() - 1]) {
                Some(h) => {
                    out.push(Item::Start(h.clone()));
                    if h.size == 0 {
                        out.push(Item::Done(h, Vec::new()));
                    } else {
                        self.file = Some((h, Vec::new()));
                    }
                }
                None => push_text(&mut out, &line),
            }
        }
    }
}

fn push_text(out: &mut Vec<Item>, bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }
    let text = String::from_utf8_lossy(bytes).to_string();
    match out.last_mut() {
        Some(Item::Text(t)) => t.push_str(&text),
        _ => out.push(Item::Text(text)),
    }
}

fn parse_header(b: &[u8]) -> Option<Header> {
    let s = std::str::from_utf8(b).ok()?;
    let mut it = s.splitn(3, ' ');
    let kind = match it.next()? {
        "file" => Kind::File,
        "img" => Kind::Image,
        _ => return None,
    };
    let size: usize = it.next()?.parse().ok()?;
    if size > MAX_SIZE {
        return None;
    }
    let name = it.next().unwrap_or("").to_string();
    Some(Header { kind, size, name })
}

// ---------------------------------------------------------------------------
// Pictures in strips
// ---------------------------------------------------------------------------

/// Largest picture sent, pixels (kept small: a picture at 0 dB moves at a
/// few hundred bytes a minute).
pub const MAX_W: u32 = 320;
pub const MAX_H: u32 = 240;
/// Horizontal strips, each its own JPEG, so the picture fills in as the
/// bytes arrive.
const STRIPS: u32 = 8;
const QUALITY: u8 = 55;

/// `img` scaled to fit [`MAX_W`] × [`MAX_H`] (height a multiple of the
/// strip count) and coded as: width and height (u16, big-endian), the
/// strip count (u8), then per strip a 2-byte length and a JPEG.
pub fn image_payload(img: &image::DynamicImage) -> Result<Vec<u8>, String> {
    let img = img
        .resize(MAX_W, MAX_H, image::imageops::FilterType::Triangle)
        .to_rgb8();
    let (w, h) = (img.width(), img.height() / STRIPS * STRIPS);
    if w == 0 || h == 0 {
        return Err("picture too small".into());
    }
    let sh = h / STRIPS;
    let mut out = Vec::new();
    out.extend((w as u16).to_be_bytes());
    out.extend((h as u16).to_be_bytes());
    out.push(STRIPS as u8);
    for k in 0..STRIPS {
        let strip = image::imageops::crop_imm(&img, 0, k * sh, w, sh).to_image();
        let mut jpg = Cursor::new(Vec::new());
        image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpg, QUALITY)
            .encode_image(&strip)
            .map_err(|e| e.to_string())?;
        let jpg = jpg.into_inner();
        out.extend((jpg.len() as u16).to_be_bytes());
        out.extend(jpg);
    }
    Ok(out)
}

/// The picture from the first bytes of an [`image_payload`]: the strips
/// whose bytes are all in, the rest grey. `None` before the size is known.
/// Returns it and (strips decoded, strips).
pub fn image_so_far(buf: &[u8]) -> Option<(image::RgbImage, (usize, usize))> {
    if buf.len() < 5 {
        return None;
    }
    let w = u16::from_be_bytes([buf[0], buf[1]]) as u32;
    let h = u16::from_be_bytes([buf[2], buf[3]]) as u32;
    let strips = buf[4] as u32;
    if w == 0 || h == 0 || strips == 0 || w > 4096 || h > 4096 {
        return None;
    }
    let sh = h / strips;
    let mut img = image::RgbImage::from_pixel(w, h, image::Rgb([38, 38, 42]));
    let (mut at, mut k) = (5, 0u32);
    while k < strips && at + 2 <= buf.len() {
        let n = u16::from_be_bytes([buf[at], buf[at + 1]]) as usize;
        if at + 2 + n > buf.len() {
            break;
        }
        if let Ok(s) = image::load_from_memory_with_format(&buf[at + 2..at + 2 + n], image::ImageFormat::Jpeg) {
            image::imageops::replace(&mut img, &s.to_rgb8(), 0, (k * sh) as i64);
        }
        at += 2 + n;
        k += 1;
    }
    Some((img, (k as usize, strips as usize)))
}

/// A file to send: a picture (by its extension, re-coded in strips) or any
/// file as it is. Returns the bytes for the stream, its kind and name.
pub fn prepare(path: &std::path::Path) -> Result<(Vec<u8>, Kind, String), String> {
    let name = path
        .file_name()
        .map_or("file".into(), |n| n.to_string_lossy().to_string());
    let ext = path
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    let data = std::fs::read(path).map_err(|e| format!("{name}: {e}"))?;
    if matches!(ext.as_str(), "jpg" | "jpeg" | "png" | "bmp" | "gif" | "webp")
        && let Ok(img) = image::load_from_memory(&data)
    {
        let stem = path
            .file_stem()
            .map_or("picture".into(), |n| n.to_string_lossy().to_string());
        let payload = image_payload(&img)?;
        return Ok((
            frame(Kind::Image, &format!("{stem}.jpg"), &payload),
            Kind::Image,
            format!("{stem}.jpg"),
        ));
    }
    if data.len() > MAX_SIZE {
        return Err(format!("{name}: larger than {} MB", MAX_SIZE >> 20));
    }
    Ok((frame(Kind::File, &name, &data), Kind::File, name))
}

/// Where received files are saved: `Documents\Ion` (made if missing),
/// under a name not yet taken.
pub fn save_received(name: &str, data: &[u8], image: Option<&image::RgbImage>) -> Result<std::path::PathBuf, String> {
    let home = std::env::var_os("USERPROFILE")
        .or_else(|| std::env::var_os("HOME"))
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| ".".into());
    let dir = home.join("Documents").join("Ion");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let clean: String = name
        .chars()
        .map(|c| {
            if c.is_alphanumeric() || matches!(c, '.' | '-' | '_') {
                c
            } else {
                '_'
            }
        })
        .collect();
    let clean = if clean.trim_matches('.').is_empty() {
        "received".to_string()
    } else {
        clean
    };
    let path = std::path::Path::new(&clean);
    let (stem, ext) = (
        path.file_stem()
            .map_or("received".into(), |s| s.to_string_lossy().to_string()),
        path.extension()
            .map(|e| format!(".{}", e.to_string_lossy()))
            .unwrap_or_default(),
    );
    let mut target = dir.join(&clean);
    let mut k = 2;
    while target.exists() {
        target = dir.join(format!("{stem}-{k}{ext}"));
        k += 1;
    }
    match image {
        Some(img) => img.save(&target).map_err(|e| e.to_string())?,
        None => std::fs::write(&target, data).map_err(|e| e.to_string())?,
    }
    Ok(target)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_and_files_come_apart_in_any_pieces() {
        let mut stream = b"hello\n".to_vec();
        stream.extend(frame(Kind::File, "notes.txt", b"line one\nline two\n"));
        stream.extend(b"after\n");
        stream.extend(frame(Kind::File, "empty", b""));
        for piece in [1, 3, 7, 1000] {
            let mut d = Demux::new();
            let mut items = Vec::new();
            for c in stream.chunks(piece) {
                items.extend(d.feed(c));
            }
            let text: String = items
                .iter()
                .filter_map(|i| if let Item::Text(t) = i { Some(t.as_str()) } else { None })
                .collect();
            assert_eq!(text, "hello\nafter\n", "pieces of {piece}");
            let done: Vec<(String, Vec<u8>)> = items
                .iter()
                .filter_map(|i| {
                    if let Item::Done(h, b) = i {
                        Some((h.name.clone(), b.clone()))
                    } else {
                        None
                    }
                })
                .collect();
            assert_eq!(
                done,
                vec![
                    ("notes.txt".into(), b"line one\nline two\n".to_vec()),
                    ("empty".into(), Vec::new())
                ]
            );
        }
    }

    #[test]
    fn a_stray_separator_is_text() {
        let mut d = Demux::new();
        let items = d.feed(b"a\x1Eb\n");
        assert_eq!(items, vec![Item::Text("a\u{1e}b\n".into())]);
    }

    #[test]
    fn a_picture_fills_in_strip_by_strip() {
        let img = image::DynamicImage::ImageRgb8(image::RgbImage::from_fn(400, 300, |x, y| {
            image::Rgb([x as u8, y as u8, 128])
        }));
        let p = image_payload(&img).unwrap();
        let (full, (k, n)) = image_so_far(&p).unwrap();
        assert_eq!((full.width(), full.height(), k, n), (320, 240, 8, 8));
        let (_, (k, _)) = image_so_far(&p[..p.len() / 2]).unwrap();
        assert!(k > 0 && k < 8, "{k}");
        assert!(image_so_far(&p[..3]).is_none());
    }
}
