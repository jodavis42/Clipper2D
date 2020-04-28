pushd %~dp0

set folder_name="Generated/VS2019_MSVC_Windows"
rd %folder_name% /s/q

mkdir %folder_name%

cd %folder_name%

set config=Windows_VS_2019

REM cmake -E remove_directory ..\..\BuildOutput\Out\%config%

cmake -G "Visual Studio 16 2019" ../../..

popd
