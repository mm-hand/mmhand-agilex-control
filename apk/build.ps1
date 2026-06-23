$ErrorActionPreference = "Stop"
$apk = Split-Path -Parent $MyInvocation.MyCommand.Path
$tools = (Resolve-Path "$apk\..\toolchains").Path
$sdk = "$tools\android-sdk"
$buildTools = "$sdk\build-tools\32.0.0"
$platform = "$sdk\platforms\android-31\android.jar"
$env:RUSTUP_HOME = "$tools\rustup"
$env:CARGO_HOME = "$tools\cargo"
$env:JAVA_HOME = (Resolve-Path "$tools\jdk\jdk-17.0.19+10").Path
$env:ANDROID_HOME = $sdk
$env:ANDROID_NDK_HOME = (Resolve-Path "$sdk\ndk\25.1.8937393").Path
$env:PATH = "$tools\cargo\bin;$env:JAVA_HOME\bin;$sdk\platform-tools;$env:PATH"

& "$tools\cargo\bin\cargo.exe" apk build
if ($LASTEXITCODE) { throw "cargo apk build failed" }

$work = "$apk\target\package"
$classes = "$work\classes"
$dex = "$work\dex"
$stage = "$work\stage"
Remove-Item $work -Recurse -Force -ErrorAction SilentlyContinue
New-Item $classes, $dex, "$stage\lib\arm64-v8a" -ItemType Directory -Force | Out-Null

& "$env:JAVA_HOME\bin\javac.exe" -source 8 -target 8 -bootclasspath $platform -d $classes `
    "$apk\java\com\mmhand\pico_head_relative_pose\MainActivity.java"
if ($LASTEXITCODE) { throw "javac failed" }
$classesJar = "$work\classes.jar"
& "$env:JAVA_HOME\bin\jar.exe" cf $classesJar -C $classes .
if ($LASTEXITCODE) { throw "jar failed" }
& "$env:JAVA_HOME\bin\java.exe" -cp "$buildTools\lib\d8.jar" com.android.tools.r8.D8 `
    --min-api 26 --lib $platform --output $dex $classesJar
if ($LASTEXITCODE) { throw "d8 failed" }

Copy-Item "$dex\classes.dex" $stage
Copy-Item "$apk\runtime_libs\arm64-v8a\libopenxr_loader.so" "$stage\lib\arm64-v8a"
Copy-Item "$apk\target\aarch64-linux-android\debug\libpico_head_relative_pose.so" "$stage\lib\arm64-v8a"

$unaligned = "$work\unsigned-unaligned.apk"
$aligned = "$work\unsigned.apk"
& "$buildTools\aapt.exe" package -f -M "$apk\AndroidManifest.xml" -I $platform -F $unaligned
if ($LASTEXITCODE) { throw "aapt package failed" }
Push-Location $stage
& "$buildTools\aapt.exe" add $unaligned classes.dex lib/arm64-v8a/libopenxr_loader.so lib/arm64-v8a/libpico_head_relative_pose.so
Pop-Location
if ($LASTEXITCODE) { throw "aapt add failed" }
& "$buildTools\zipalign.exe" -f 4 $unaligned $aligned
if ($LASTEXITCODE) { throw "zipalign failed" }

$out = "$apk\generated_apk\pico_head_relative_pose.apk"
$keystore = "$tools\user-home\.android\debug.keystore"
& "$buildTools\apksigner.bat" sign --v4-signing-enabled false --ks $keystore --ks-pass pass:android `
    --key-pass pass:android --out $out $aligned
if ($LASTEXITCODE) { throw "apksigner failed" }
& "$buildTools\apksigner.bat" verify $out
if ($LASTEXITCODE) { throw "APK verification failed" }
Write-Output $out
