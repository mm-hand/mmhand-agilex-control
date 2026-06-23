@echo off
setlocal
C:\Users\73547\.pixi\bin\pixi.exe run --manifest-path C:\pixi_ws\pixi.toml cmd /d /s /c ""call C:\pixi_ws\ros2-windows\local_setup.bat && python "%~dp0pc\run.py" %*""
exit /b %ERRORLEVEL%
