#!/usr/bin/env bash
set -euo pipefail
apk="$(cd "$(dirname "$0")" && pwd)"

# Toolchain locations (system-level install, overridable via env).
: "${JAVA_HOME:=/usr/lib/jvm/java-17-openjdk-amd64}"
: "${ANDROID_HOME:=$HOME/Android/Sdk}"
: "${ANDROID_NDK_HOME:=$ANDROID_HOME/ndk/25.1.8937393}"
export JAVA_HOME ANDROID_HOME ANDROID_NDK_HOME
build_tools="$ANDROID_HOME/build-tools/32.0.0"
platform="$ANDROID_HOME/platforms/android-31/android.jar"
export PATH="$JAVA_HOME/bin:$ANDROID_HOME/platform-tools:$PATH"

# 1) Cross-compile the Rust shared library.
cargo apk build

work="$apk/target/package"
classes="$work/classes"
dex="$work/dex"
stage="$work/stage"
rm -rf "$work"
mkdir -p "$classes" "$dex" "$stage/lib/arm64-v8a"

# 2) Compile MainActivity.java -> classes.jar -> dex.
javac -source 8 -target 8 -bootclasspath "$platform" -d "$classes" \
    "$apk/java/com/mmhand/pico_head_relative_pose/MainActivity.java"
jar cf "$work/classes.jar" -C "$classes" .
java -cp "$build_tools/lib/d8.jar" com.android.tools.r8.D8 \
    --min-api 26 --lib "$platform" --output "$dex" "$work/classes.jar"

# 3) Stage the payload.
cp "$dex/classes.dex" "$stage"
cp "$apk/runtime_libs/arm64-v8a/libopenxr_loader.so" "$stage/lib/arm64-v8a"
cp "$apk/target/aarch64-linux-android/debug/libpico_head_relative_pose.so" "$stage/lib/arm64-v8a"

# 4) Package and align.
unaligned="$work/unsigned-unaligned.apk"
aligned="$work/unsigned.apk"
"$build_tools/aapt" package -f -M "$apk/AndroidManifest.xml" -I "$platform" -F "$unaligned"
( cd "$stage" && "$build_tools/aapt" add "$unaligned" \
    classes.dex lib/arm64-v8a/libopenxr_loader.so lib/arm64-v8a/libpico_head_relative_pose.so )
"$build_tools/zipalign" -f 4 "$unaligned" "$aligned"

# 5) Sign with the debug keystore (create it if missing).
keystore="$HOME/.android/debug.keystore"
if [ ! -f "$keystore" ]; then
    keytool -genkeypair -v -keystore "$keystore" \
        -storepass android -keypass android -alias androiddebugkey \
        -keyalg RSA -keysize 2048 -validity 10000 \
        -dname "CN=Android Debug,O=Android,C=US"
fi

out="$apk/generated_apk/pico_head_relative_pose.apk"
"$build_tools/apksigner" sign --v4-signing-enabled false \
    --ks "$keystore" --ks-pass pass:android --key-pass pass:android --out "$out" "$aligned"
"$build_tools/apksigner" verify "$out"
echo "$out"
