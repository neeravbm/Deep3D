# Install Leptonica

```
cd /Users/neeravbm/Documents/libs
mkdir leptonica
cd leptonica
git clone https://github.com/DanBloomberg/leptonica.git src
mkdir -p builds/osx-shared-release-clang
cd builds/osx-shared-release-clang
cmake -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS:BOOL=ON ../../src
make -j8
make install
```

# Install Tesseract

```
cd /Users/neeravbm/Documents/libs
mkdir tesseract
cd tesseract
git clone https://github.com/tesseract-ocr/tesseract.git src
mkdir -p builds/osx-shared-release-clang
cd builds/osx-shared-release-clang
cmake -DCMAKE_INSTALL_PREFIX=dist -DCMAKE_BUILD_TYPE=Release -DBUILD_TRAINING_TOOLS:BOOL=OFF -DBUILD_SHARED_LIBS:BOOL=ON -DLeptonica_DIR=/Users/neeravbm/Documents/libs/leptonica/builds/osx-shared-release-clang/dist/cmake ../../src
make -j8
make install

cd /Users/neeravbm/Documents/libs/tesseract
mkdir data
cd data
wget https://github.com/tesseract-ocr/tessdata/raw/master/eng.traineddata
