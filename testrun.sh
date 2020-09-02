rm -rf build
mkdir build
cd build
cmake ..
make
./3D_object_tracking SHITOMASI SIFT MAT_BF  DES_HOG  
./3D_object_tracking SHITOMASI BRISK MAT_BF DES_BINARY
./3D_object_tracking SHITOMASI BRIEF MAT_BF DES_BINARY
./3D_object_tracking SHITOMASI ORB MAT_BF   DES_BINARY
./3D_object_tracking SHITOMASI FREAK MAT_BF DES_BINARY

# Test FAST Detector with various Descriptors
./3D_object_tracking FAST BRISK MAT_BF DES_BINARY
./3D_object_tracking FAST BRIEF MAT_BF DES_BINARY
./3D_object_tracking FAST ORB   MAT_BF DES_BINARY
./3D_object_tracking FAST FREAK MAT_BF DES_BINARY

#Test BRISK Detector with various Descriptors
./3D_object_tracking BRISK BRISK MAT_BF DES_BINARY
./3D_object_tracking BRISK BRIEF MAT_BF DES_BINARY
./3D_object_tracking BRISK ORB   MAT_BF DES_BINARY
./3D_object_tracking BRISK AKAZE MAT_BF DES_BINARY

#Test ORB Detector with various Descriptors
./3D_object_tracking ORB BRISK MAT_BF DES_BINARY
./3D_object_tracking ORB BRIEF MAT_BF DES_BINARY
./3D_object_tracking ORB ORB   MAT_BF DES_BINARY
./3D_object_tracking ORB FREAK MAT_BF DES_BINARY

#Test AKAZE Detector with various Descriptors
./3D_object_tracking AKAZE BRISK MAT_BF DES_BINARY
./3D_object_tracking AKAZE BRIEF MAT_BF DES_BINARY
./3D_object_tracking AKAZE ORB   MAT_BF DES_BINARY
./3D_object_tracking AKAZE FREAK MAT_BF DES_BINARY
./3D_object_tracking AKAZE AKAZE MAT_BF DES_BINARY

#Test SIFT Detector with various Descriptors
./3D_object_tracking SIFT BRISK MAT_BF DES_BINARY
./3D_object_tracking SIFT BRIEF MAT_BF DES_BINARY
./3D_object_tracking SIFT ORB   MAT_BF DES_BINARY
./3D_object_tracking SIFT FREAK MAT_BF DES_BINARY
./3D_object_tracking SIFT SIFT  MAT_BF DES_HOG
