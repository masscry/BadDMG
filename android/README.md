# BadDMG Android port

The project is a regular Android Gradle project. The native code is built from
the top-level `CMakeLists.txt`, which also fetches SDL3; on Android the `BadDMG`
target becomes `libmain.so` and the Java `SDLActivity` starts it.

## ROMs and saves

`RomPickerActivity` is the launcher and ROM library. It lists the ROMs in
`<app files>/roms/`; the first launch with an empty list opens the system file
picker. Tapping a ROM starts `MainActivity` (`SDLActivity`) with `-c <path>`,
and Back returns to the list. "Add ROM" imports another ROM; identical ROM
contents are deduplicated and reuse the existing entry and saves. Long-press a
ROM for "Add to home screen" (a pinned shortcut), "Export save", "Import
save" and "Delete ROM" (with or without its save files); saves are exported
as a ZIP with the `.sav` and `.rtc` files. Saves are
memory-mapped next to the imported ROM, as on desktop, and named by the ROM's
SHA-256, so re-importing the same ROM keeps its progress. Only one emulator
instance runs at a time. The "Change ROM" launcher shortcut (long-press the
app icon) opens the picker directly.

## SDL Java shim

The `org.libsdl.app` classes are not tracked in this repository. `app/build.gradle`
downloads the SDL archive pinned in `deps/CMakeLists.txt` (same commit), verifies
its SHA-256 and unpacks `android-project/app/src/main/java` into
`app/build/generated/sdl/java`, which is added to the Java source set. Bump the
URL and hash in `app/build.gradle` together with the SDL pin; nothing needs to
be refreshed by hand.

## Release builds

`./gradlew assembleRelease` builds optimized native libraries
(`RelWithDebInfo`, i.e. `-O2 -g -DNDEBUG`) but no signing config is committed,
so the APK comes out unsigned. Sign it with your keystore (Android Studio:
*Build > Generate Signed Bundle / APK*) before distributing.

For local testing, `./install.sh` builds the release APK for the connected
device's ABI, signs it with the debug key and installs it with
`adb install -r` (app data is kept). The debug key is local-only; use a real
keystore for distribution.

## Toolchain

`app/build.gradle` pins NDK 29.0.14206865 and CMake 4.1.2, both installable
through Android Studio's SDK Manager. Builds target `armeabi-v7a` (older
32-bit devices), `arm64-v8a` and `x86_64` (the emulator); adjust `abiFilters`
if you need other devices.
