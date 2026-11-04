#!/usr/bin/env bash
# Build a release APK for the connected device, sign it with the local debug
# key and install it. Existing app data is kept.
set -euo pipefail
cd "$(dirname "$0")"

# No system Java on this machine; Android Studio bundles one.
export JAVA_HOME="${JAVA_HOME:-/Applications/Android Studio.app/Contents/jbr/Contents/Home}"
SDK_DIR="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-$(sed -n 's/^sdk.dir=//p' local.properties)}}"
ADB="$SDK_DIR/platform-tools/adb"
APKSIGNER="$(ls -d "$SDK_DIR"/build-tools/* | sort | tail -1)/apksigner"

ABI="$("$ADB" shell getprop ro.product.cpu.abi | tr -d '\r')"
./gradlew :app:assembleRelease -Pabi="$ABI"

# shortcut: debug key for local installs, add a real signingConfig for distribution
"$APKSIGNER" sign \
    --ks "$HOME/.android/debug.keystore" --ks-pass pass:android \
    --ks-key-alias androiddebugkey --key-pass pass:android \
    --out app/build/outputs/apk/release/app-release-signed.apk \
    app/build/outputs/apk/release/app-release-unsigned.apk

"$ADB" install -r app/build/outputs/apk/release/app-release-signed.apk
