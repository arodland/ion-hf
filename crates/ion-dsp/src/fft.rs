//! Mixed-radix complex FFT (radix 4, 2 and generic odd factors).

use crate::dsp::Complex;

pub struct Fft {
    n: usize,
    /// (radix p, remaining length m) per stage.
    stages: Vec<(usize, usize)>,
    /// e^{-2πik/n}.
    tw: Vec<Complex>,
}

impl Fft {
    pub fn new(n: usize) -> Self {
        assert!(n >= 1);
        let mut stages = Vec::new();
        let mut rest = n;
        let mut push = |p: usize, rest: &mut usize| {
            *rest /= p;
            stages.push((p, *rest));
        };
        while rest.is_multiple_of(4) {
            push(4, &mut rest);
        }
        while rest.is_multiple_of(2) {
            push(2, &mut rest);
        }
        let mut p = 3;
        while rest > 1 {
            while rest.is_multiple_of(p) {
                push(p, &mut rest);
            }
            p += 2;
        }
        let tw = (0..n)
            .map(|k| {
                let a = -core::f64::consts::TAU * k as f64 / n as f64;
                Complex::new(a.cos() as f32, a.sin() as f32)
            })
            .collect();
        Self { n, stages, tw }
    }

    /// Forward transform (e^{-j…}), unnormalised, from `input` into `out`.
    pub fn forward(&self, input: &[Complex], out: &mut [Complex]) {
        assert_eq!(input.len(), self.n);
        assert_eq!(out.len(), self.n);
        if self.n == 1 {
            out[0] = input[0];
            return;
        }
        let mut scratch = Vec::new();
        self.work(out, input, 0, 1, &mut scratch);
    }

    /// Inverse transform (e^{+j…}), unnormalised.
    pub fn inverse(&self, input: &[Complex], out: &mut [Complex]) {
        let conj: Vec<Complex> = input.iter().map(|c| c.conj()).collect();
        self.forward(&conj, out);
        for c in out.iter_mut() {
            *c = c.conj();
        }
    }

    fn work(&self, out: &mut [Complex], f: &[Complex], stage: usize, fstride: usize, scratch: &mut Vec<Complex>) {
        let (p, m) = self.stages[stage];
        if m == 1 {
            for q in 0..p {
                out[q] = f[q * fstride];
            }
        } else {
            for q in 0..p {
                self.work(
                    &mut out[q * m..(q + 1) * m],
                    &f[q * fstride..],
                    stage + 1,
                    fstride * p,
                    scratch,
                );
            }
        }
        match p {
            2 => self.bfly2(out, fstride, m),
            4 => self.bfly4(out, fstride, m),
            _ => self.bfly_generic(out, fstride, p, m, scratch),
        }
    }

    fn bfly2(&self, out: &mut [Complex], fstride: usize, m: usize) {
        for k in 0..m {
            let t = out[k + m] * self.tw[k * fstride];
            out[k + m] = out[k] - t;
            out[k] += t;
        }
    }

    fn bfly4(&self, out: &mut [Complex], fstride: usize, m: usize) {
        for k in 0..m {
            let s0 = out[k + m] * self.tw[k * fstride];
            let s1 = out[k + 2 * m] * self.tw[2 * k * fstride];
            let s2 = out[k + 3 * m] * self.tw[3 * k * fstride];
            let s5 = out[k] - s1;
            let f0 = out[k] + s1;
            let s3 = s0 + s2;
            let s4 = s0 - s2;
            out[k + 2 * m] = f0 - s3;
            out[k] = f0 + s3;
            out[k + m] = Complex::new(s5.re + s4.im, s5.im - s4.re);
            out[k + 3 * m] = Complex::new(s5.re - s4.im, s5.im + s4.re);
        }
    }

    fn bfly_generic(&self, out: &mut [Complex], fstride: usize, p: usize, m: usize, scratch: &mut Vec<Complex>) {
        let n = self.n;
        scratch.resize(p, Complex::ZERO);
        for u in 0..m {
            for q in 0..p {
                scratch[q] = out[u + q * m];
            }
            for q1 in 0..p {
                let k = u + q1 * m;
                let mut acc = scratch[0];
                let mut twidx = 0usize;
                let step = fstride * k % n;
                for s in scratch.iter().take(p).skip(1) {
                    twidx += step;
                    if twidx >= n {
                        twidx -= n;
                    }
                    acc += *s * self.tw[twidx];
                }
                out[k] = acc;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dft(x: &[Complex]) -> Vec<Complex> {
        let n = x.len();
        (0..n)
            .map(|k| {
                let mut re = 0f64;
                let mut im = 0f64;
                for (i, v) in x.iter().enumerate() {
                    let a = -core::f64::consts::TAU * (i * k % n) as f64 / n as f64;
                    re += v.re as f64 * a.cos() - v.im as f64 * a.sin();
                    im += v.re as f64 * a.sin() + v.im as f64 * a.cos();
                }
                Complex::new(re as f32, im as f32)
            })
            .collect()
    }

    #[test]
    fn matches_dft() {
        for &n in &[1usize, 2, 3, 4, 5, 8, 12, 30, 36, 60, 64, 120, 7, 49] {
            let x: Vec<Complex> = (0..n)
                .map(|i| Complex::new((i as f32 * 0.37).sin(), (i as f32 * 1.1).cos()))
                .collect();
            let fft = Fft::new(n);
            let mut y = vec![Complex::ZERO; n];
            fft.forward(&x, &mut y);
            let d = dft(&x);
            for k in 0..n {
                assert!((y[k] - d[k]).norm() < 1e-3 * (n as f32).sqrt(), "n={n} k={k}");
            }
            let mut z = vec![Complex::ZERO; n];
            fft.inverse(&y, &mut z);
            for k in 0..n {
                assert!((z[k].scale(1.0 / n as f32) - x[k]).norm() < 1e-4, "inv n={n}");
            }
        }
    }
}
