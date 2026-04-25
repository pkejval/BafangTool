# BafangTool

Desktop/web configuration tool for Bafang UART controllers with focus on M400 (36V).

## Features

- Read and write `Basic`, `Pedal`, and `Throttle` parameter blocks
- Controller-bound profiles (profiles are cryptographically tied to detected controller identity)
- Raw and experimental modes for advanced diagnostics
- Live telemetry and error-code readout
- Safe write mode: only changed parameters are accepted, based on first controller read snapshot

## Supported Hardware

- Primary target: Bafang `M400 UART`
- Also works with many Bafang UART-compatible controllers where command blocks match

## Safety Model

- The tool does **not** hard-limit values in UI for power users
- Controller firmware remains the final validator on write
- Writes are guarded:
  - You must read from controller first
  - Only parameters present in the first read are eligible for write
  - Only changed values are accepted into write payload logic

## Requirements

- Python 3.10+
- USB TTL adapter (3.3V/5V as required by your cable/controller)
- Bafang programming cable (UART)

## Quick Start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Open `http://localhost:5000`.

## Wiring Reminder

Typical UART cable mapping:

- USB-TTL TX -> Controller RX
- USB-TTL RX -> Controller TX
- USB-TTL GND -> Controller GND

Check your exact cable/controller documentation before powering on.

## Profiles

- Profiles are stored as JSON metadata + parameter payload
- Each profile is bound to detected controller identity fingerprint (`manufacturer/model/HW/FW/...`)
- Applying profile to a different controller returns HTTP `409` with mismatch details

## Build Executables

Local (same-OS build) with PyInstaller:

```bash
pip install pyinstaller
pyinstaller --noconfirm --clean --onedir --name BafangTool app.py \
  --add-data "templates:templates" \
  --add-data "bafang:bafang"
```

For Windows, run the build on Windows.

## CI/CD Releases

GitHub Actions workflow builds release artifacts for:

- Linux
- Windows
- macOS
- Android APK for USB OTG USB-to-Serial adapters

Release is created automatically on Git tags like `v1.2.0`.

## Android APK

The Android app is native and uses Android USB Host mode. Connect the Bafang programming cable through a USB OTG adapter, grant USB permission, then use the app to connect and read controller blocks.

Local APK build:

```bash
gradle assembleRelease
```

The CI release artifact is named `BafangTool-<tag>-android.apk`.

## Legal / Warranty

Use at your own risk. Incorrect configuration can damage hardware, void warranty, or violate local regulations.
