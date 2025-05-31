
#pragma once

#include <string>

class CCycQTSkeleton;

class CCycQTImage
{
public:
    CCycQTImage(CCycQTSkeleton* _qt_skeleton);
    ~CCycQTImage();
    
    /* Meaning 32bits per pixel, unpadded. */
    int display_rgba(void* data, int width, int height, const std::string& title);
    /* -1 means no key was pressed */
    int get_last_key(const std::string& title);

private:
    std::shared_ptr<CCycQTSkeleton> m_CcrQTSkeleton = nullptr;
};
