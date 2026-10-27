@echo off
setlocal
rem Explicit opt-in: overwrites results and exported policies in this release copy.
pushd "%~dp0..\reinforcement-learning" || exit /b 1
python --version >nul 2>&1
if errorlevel 1 goto :failed
python src\train.py --episodes 6000 --seed 0
if errorlevel 1 goto :failed
python src\evaluate.py --episodes 300 --seed 12345 --save
if errorlevel 1 goto :failed
python src\export_policy.py
if errorlevel 1 goto :failed
popd
exit /b 0
:failed
popd
echo Failed. Review the output above.
exit /b 1
