# PC Vision Upload Pack (Google Drive)

This folder is meant to be copied to a brand new Windows PC (or a reimaged PC) to run the SAR PC vision GUI and backend.

## What To Copy

For a brand new PC, copy this entire folder to the target PC (example: `Desktop\GOOGLE_DRIVE_PC_SETUP`), keeping the `pc-vision` folder intact.

For a PC that already has vision working, use the lightweight update flow below instead of recopying big files.

## First-Time Setup On The Target PC

1. Install Python 3.11+ from python.org.
2. Open PowerShell inside the `pc-vision` folder.
3. Run:

```powershell
.\run_vision_ui.ps1
```

The first run will create a local `.venv` and install dependencies (can take a few minutes).

If scripts are blocked, run the batch launcher:

```text
run_vision_ui.bat
```

## Live Footage Window Size

In the GUI, set `Preview window` to:

- `Fit to screen` (default, big window)
- `Fullscreen` (true fullscreen)
- `Off` (no video window)

Then click `Save Settings` and `Start Vision`.

## Per-PC Settings

This upload pack intentionally does NOT include `vision_config.json` so each PC starts clean and you configure it in the GUI.

## Lightweight Update For An Existing PC Vision Install

Use this when the other PC already has:

- the `.venv`
- Python packages already installed
- the existing `yolov8n.pt` model already present

Copy this `GOOGLE_DRIVE_PC_SETUP` folder, or a lightweight update zip made from it, onto the target PC and run:

```powershell
.\update_existing_pc_vision.ps1 -TargetPath "C:\path\to\existing\pc-vision"
```

What it does:

- updates the shipped `pc-vision` app files
- preserves that PC's `vision_config.json`
- preserves `vision_ui_state.json` if present
- does not reinstall packages
- does not copy the large `yolov8n.pt` model file

If the old install is in a common location like Desktop, Documents, or Downloads, you can also try:

```powershell
.\update_existing_pc_vision.ps1
```

Optional flags:

- `-UpdateDependencies`
  Use only if you know package requirements changed on the target PC.
- `-IncludeModel`
  Use only if the target PC is missing `yolov8n.pt`.

## Creating A Lightweight Update Zip

If you want a smaller handoff package for the already-configured PC, run:

```powershell
.\create_lightweight_update_zip.ps1
```

When this folder still sits inside the main project repo, that command now auto-syncs the latest files from the sibling `..\pc-vision` folder first, so the zip picks up your newest Python changes.

This creates:

```text
SAR_PC_Vision_Lightweight_Update.zip
```

That zip includes:

- the updater script
- updated `pc-vision` code and launchers
- no `.venv`
- no `yolov8n.pt`

## How To Upload Or Send It To The Other PC

Use the generated lightweight zip:

```text
SAR_PC_Vision_Lightweight_Update.zip
```

You can move it to the other PC using any of these:

- upload it to Google Drive, then download it on the other PC
- send it through chat or email
- copy it with a USB drive
- copy it over LAN/shared folder

Because this is the lightweight package, it is meant only to update code and launchers on a PC that already has:

- the existing `pc-vision` folder
- the existing `.venv`
- the existing `yolov8n.pt` model

### On The Other PC

1. Extract the zip anywhere.
2. Open PowerShell in the extracted folder.
3. Run:

```powershell
.\update_existing_pc_vision.ps1 -TargetPath "C:\path\to\existing\pc-vision"
```

4. Then launch the existing install normally:

```powershell
.\run_vision_ui.ps1
```
