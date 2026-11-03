# BadDMG Android port

The project is a regular Android Gradle project. The native code is built from
the top-level `CMakeLists.txt`, which also fetches SDL3; on Android the `BadDMG`
target becomes `libmain.so` and the Java `SDLActivity` starts it.

## ROMs and saves

`RomPickerActivity` is the launcher. On first launch it opens the system file
picker, copies the chosen ROM to `<app files>/roms/`, and starts
`MainActivity` (`SDLActivity`) with `-c <path>`. Saves (`.sav`, `.rtc`) are
memory-mapped next to the imported ROM, as on desktop. The "Change ROM"
launcher shortcut (long-press the app icon) imports another ROM.

## SDL Java shim

`app/src/main/java/org/libsdl/app/` is copied from SDL's `android-project`
template (SDL 3.2.16, the tag pinned in `deps/CMakeLists.txt`). Refresh it from
`SDL/android-project/app/src/main/java/org/libsdl/app/` when bumping SDL.

## Release builds

`./gradlew assembleRelease` builds optimized native libraries
(`RelWithDebInfo`, i.e. `-O2 -g -DNDEBUG`) but no signing config is committed,
so the APK comes out unsigned. Sign it with your keystore (Android Studio:
*Build > Generate Signed Bundle / APK*) before distributing.

## Toolchain

`app/build.gradle` pins NDK 29.0.14206865 and CMake 4.1.2, both installable
through Android Studio's SDK Manager. Builds target `armeabi-v7a` (older
32-bit devices), `arm64-v8a` and `x86_64` (the emulator); adjust `abiFilters`
if you need other devices.
