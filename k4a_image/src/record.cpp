#include "KFRRecord.h"
#include "KinectWrapper.h"
#include <iostream>

using namespace std;

// Basic playback test to check tags
int playback(){
    k4a_playback_t playback_handle = NULL;
    if (k4a_playback_open("/home/fri/Documents/henry/dynamic_tracker/k4a_image/test.mkv", &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording\n");
        return 1;
    }

    // Print a tag from this playback file
    size_t bufSize = 256;
    char* tagVal = (char*) malloc(sizeof(char) * bufSize);
    k4a_buffer_result_t buffer_result = k4a_playback_get_tag(playback_handle, "TEST", tagVal, &bufSize);
    if (buffer_result == K4A_BUFFER_RESULT_SUCCEEDED)
        printf("Tag: %s\n", tagVal);
    else if (buffer_result == K4A_BUFFER_RESULT_TOO_SMALL)
        printf("Tag too long.\n");
    else
        printf("Tag does not exist. Tag was not recorded.\n");

    k4a_playback_close(playback_handle);
}

int main(){
  KFRRecord kfrRecord("/home/fri/Documents/henry/dynamic_tracker/k4a_image/test.mkv");
  KinectWrapper kinectWrapper(0, kfrRecord);
  kfrRecord.addTag("TEST","value");
  // Capture some frames
  int n = 150;
  while(n--){
    kinectWrapper.capture();
    cout << n << endl;
  }
  kfrRecord.close();
  playback();
  return 0;
}

