@echo off
rmdir /Q /S bin
mkdir bin
pushd bin

rem Name
set name=App

rem Include directories 
set inc=/I ..\..\third_party\

rem Proj source files
set src_proj=..\source\*.c

rem Common source files
set src_common=..\..\common\*.c

rem All source together
set src_all= %src_common% %src_proj%

rem OS Libraries
set os_libs= opengl32.lib kernel32.lib user32.lib ^
shell32.lib vcruntime.lib msvcrt.lib gdi32.lib Advapi32.lib

rem Compile Release
cl /MP /FS /Ox /W0 /Fe%name%.exe %src_all% %inc% ^
/EHsc /link /SUBSYSTEM:CONSOLE /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:LIBCMT ^
%os_libs%

popd
