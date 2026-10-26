# Acoustic test: two computers, no radios

Two computers running Ion HF talk speaker to microphone across a room. No
RF, so no licence or ID questions; the room's echoes stand in for
multipath. It checks a real audio path end to end, and Windows and macOS
working together, before any on-air test.

## Each computer

1. Install Ion HF (Windows: the zip; macOS: the disk image, then
   right-click → Open the first time; allow the microphone when asked).
2. **Setup → Station:** a call. One computer N0CALL, the other N0CALL/P.
3. **Setup → Radio:** Link **Sound card**; In: the built-in microphone;
   Out: the built-in speakers. Restart Ion when it asks.
4. **Setup → Keying:** PTT **VOX** (Ion keys nothing; it just plays).
5. Turn off the computer's voice processing, which treats Ion's tones as
   noise to remove:
   - Windows 11: Settings → System → Sound → the microphone → Audio
     enhancements **Off** (and Voice clarity off, where shown); the same
     for the speakers.
   - macOS: while Ion is using the microphone, Control Centre → Mic Mode
     → **Standard** (not Voice Isolation). Input volume about 60%
     (System Settings → Sound → Input).
6. Speaker volume about half.

## The test

1. Put the computers 1–2 m apart, speakers facing each other.
2. N0CALL/P: **Listen** tab, Answer **Auto**.
3. N0CALL: Call **N0CALL/P**, **Connect**.
4. Watch both waterfalls for the bursts, and **Rpt Sent / Rpt Rcvd** for
   how well each hears the other.
5. Send a line of chat each way, then a small picture.
6. Disconnect: each plays its CW ID.

## Then push it

- Turn the volume down, or move apart: lower S:N, slower levels.
- Turn one computer away, or put it behind something: more echo.
- Run a fan or play music quietly: noise and interference (watch the band
  meter pick the clearest third).

## Keep

Screenshots of the Link tab (Chat and Log), the levels reached, anything
odd, and on Windows the log, `%APPDATA%\Ion HF\ion.log`.
