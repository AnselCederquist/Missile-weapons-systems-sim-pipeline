echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\tell.exe" DESKTOP-34L9OBC 56875 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 41028) 
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 33592) 
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 44488) 
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 36224) 
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 19320) 
if /i "%LOCALHOST%"=="DESKTOP-34L9OBC" (%KILL_CMD% 49984)
del "D:\Weapons-systems-sim-pipeline\03_nozzle_cfd\ansys\cleanup-fluent-DESKTOP-34L9OBC-19320.bat"
