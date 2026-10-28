@echo off
rem Removes NinScan from Windows COM and the ASCOM chooser. Asks for administrator rights.
"%~dp0NinScan.Driver.exe" /unregserver
