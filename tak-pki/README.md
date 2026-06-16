# TAK bridge — PKI & data packages

The ESP streams drone telemetry as Cursor-on-Target (CoT) to TAK clients. TLS is
now the **only** transport — UDP broadcast (:4242) and plain TCP (:8087) have been
removed (see `src/tak.cpp`):

| Transport | Port | Clients |
|-----------|------|---------|
| TLS stream | 8089 | OmniTAK, iTAK, TAK Aware (iOS) — import a data package. Up to `MAX_TLS_CLIENTS` (4) stream concurrently. |
| API/enroll probe | 8443, 8446 | accept-and-close so iOS clients proceed to 8089 |

iOS TAK apps won't take a raw feed — they connect to a *server* over TLS and must
trust its certificate via an imported **data package**. This folder builds that.

All three apps share the **same** client cert; the ESP does not track app identity
(it logs an active-connection *count*, not which app). Multiple apps can stream at
once.

## Regenerate everything

```bash
bash tak-pki/gen.sh
```

Produces (all git-ignored):
- `ca.* server.* client.*` — PEM keys/certs
- `caCert.p12` / `clientCert.p12` — **legacy** PKCS#12 (3DES/SHA1) bundles
- `../src/tak_cert.h` — server cert+key baked into the firmware (rebuild after)
- `dist/ESP-Drone-TAK-OmniTAK.zip` — data package for **OmniTAK**
- `dist/ESP-Drone-TAK-iTAK.zip` — data package for **iTAK**
- `dist/ESP-Drone-TAK-TAKAware.zip` — data package for **TAK Aware**

Existing certs are reused; delete `tak-pki/*.crt` to force a fresh PKI (then
re-flash and re-import the packages).

## Import (iOS)
1. Get the right `.zip` onto the phone (AirDrop → Save to Files).
2. App → **Settings → Network → Servers → ＋ → Upload Server Package** → pick it.
3. Join the ESP SoftAP, open `http://192.168.4.1`, sync clock + pick a drone.

## Hard-won format notes (why three packages)
- **OmniTAK** accepts the ATAK layout: certs in `cert/`, numbered cert keys
  (`caLocation0`…) inside `cot_streams`, manifest in `MANIFEST/`.
- **iTAK** and **TAK Aware** use the same flat layout, so they're built from one
  helper (`build_flat_package`) into two separately-named zips. They need:
  - certs + `.pref` **flat at the zip root** (no `cert/` subfolder),
  - cert keys **non-numbered** in a `com.atakmap.app_preferences` block
    (`caLocation`, `caPassword`, `certificateLocation`, `clientPassword`) —
    *not* the numbered form in `cot_streams`,
  - `manifest.xml` at the **root** (not under `MANIFEST/`),
  - PKCS#12 in **legacy** encryption (OpenSSL 3's default AES/PBKDF2 fails to import).
- Cert password is `atakatak`; the ESP does not verify the client cert.

## Reminder
`TAK_TEST_SIMULATE` in `src/tak.cpp` is **true** (drone orbits a fixed circle for
indoor testing). Set it **false** for real field flights (live GPS).
