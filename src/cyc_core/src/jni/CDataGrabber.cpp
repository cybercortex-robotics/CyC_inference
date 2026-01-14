// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#include "common.h"
#include <CCycFilterBase.h>
#include <opencv2/imgcodecs.hpp>
#include "cvplot/cvplot.h"

CCycFilterBase* getGrabberFilter(JNIEnv* env, jobject k)
{
    jclass cls = env->GetObjectClass(k);
    jfieldID fid = env->GetFieldID(cls, "mFilter", "Lai/cybercortex/testervisioncore_android/jni/CCycFilterBase;");
    jobject filter = env->GetObjectField(k, fid);

    return getPtrFromAddressField<CCycFilterBase>(env, filter);
}

DEF_METHOD(jbyteArray, CDataGrabber, getImageJpegEncoded)(JNIEnv* env, jobject k, jint quality)
{
    auto filter = getGrabberFilter(env, k);
    jbyteArray encodedImage;

    switch (filter->getOutputDataType())
    {
        case CyC_IMAGE:
        {
            CycImages images;
            if (!filter->getData(images) || images.empty())
            {
                throwRuntimeException(env,
                        fmt::format("Could not get images from filter [{}].", filter->getFilterKey().nFilterID).c_str());
            }
            else
            {
                cv::Mat tmpImg{ (int)images.front().nRows, (int)images.front().nCols, (int)images.front().nType1, images.front().pData1 };
                if (tmpImg.empty())
                {
                    throwRuntimeException(env,
                            fmt::format("Could not convert images from filter [{}].", filter->getFilterKey().nFilterID).c_str());
                }

                std::vector<int> params;
                params.push_back(cv::IMWRITE_JPEG_QUALITY);
                params.push_back(quality); //image quality

                std::vector<uchar> image_buffer;
                if (!cv::imencode(".jpg", tmpImg, image_buffer, params))
                {
                    throwRuntimeException(env,
                            fmt::format("Could not encode image from filter [{}].", filter->getFilterKey().nFilterID).c_str());
                }

                encodedImage = env->NewByteArray(image_buffer.size());
                env->SetByteArrayRegion(encodedImage, 0, image_buffer.size(), reinterpret_cast<const jbyte*>(image_buffer.data()));
            }
        }
        break;
        case CyC_IMU:
        {
            std::vector<std::string> signals_names;
            signals_names.push_back("X acc. [nr. of g = 9.81m/s^2]");
            signals_names.push_back("Y acc. [nr. of g = 9.81m/s^2]");
            signals_names.push_back("Z acc. [nr. of g = 9.81m/s^2]");
            signals_names.push_back("X ang. velocity [rad/s]");
            signals_names.push_back("Y ang. velocity [rad/s]");
            signals_names.push_back("Z ang. velocity [rad/s]");
            signals_names.push_back("Roll [deg]");
            signals_names.push_back("Pitch [deg]");
            signals_names.push_back("Yaw [deg]");

            static CCycPlot ccr_plot(": Inertial Measurement Unit", cv::Size(1000, 500), signals_names);

            CycImu meas;
            if (!filter->getData(meas))
            {
                throwRuntimeException(env,
                                      fmt::format("Could not get data from filter [{}].", filter->getFilterKey().nFilterID).c_str());
            }
            else
            {
                std::vector<float> signals_values;
                signals_values.push_back(meas.accX);
                signals_values.push_back(meas.accY);
                signals_values.push_back(meas.accZ);
                signals_values.push_back(meas.gyroX);
                signals_values.push_back(meas.gyroY);
                signals_values.push_back(meas.gyroZ);
                signals_values.push_back(meas.roll);
                signals_values.push_back(meas.pitch);
                signals_values.push_back(meas.yaw);

                CyC_INT nr_of_fps = 1000;

                cv::Mat tmp_plot_img;
                ccr_plot.plot(signals_values, 0.33F, nr_of_fps, tmp_plot_img);

                if (tmp_plot_img.empty())
                {
                    throwRuntimeException(env,
                                          fmt::format("Could not get plot image from filter [{}].", filter->getFilterKey().nFilterID).c_str());
                }

                std::vector<int> params;
                params.push_back(cv::IMWRITE_JPEG_QUALITY);
                params.push_back(quality); //image quality

                std::vector<uchar> image_buffer;
                if (!cv::imencode(".jpg", tmp_plot_img, image_buffer, params))
                {
                    throwRuntimeException(env,
                                          fmt::format("Could not encode image from filter [{}].", filter->getFilterKey().nFilterID).c_str());
                }

                encodedImage = env->NewByteArray(image_buffer.size());
                env->SetByteArrayRegion(encodedImage, 0, image_buffer.size(), reinterpret_cast<const jbyte*>(image_buffer.data()));
            }
        }
        break;
        default:
        {
            throwRuntimeException(env,
                    fmt::format("Filter [{}] does not output images.", filter->getFilterKey().nFilterID).c_str());
        }
        break;
    }

    return encodedImage;
}

DEF_METHOD(jintArray, CDataGrabber, getIntArray)(JNIEnv* env, jobject k)
{
    throwRuntimeException(env, "Method not implemented. Check jni/CDataGrabber.cpp for getIntArray.");
    return nullptr;
}

DEF_METHOD(jfloatArray, CDataGrabber, getFloatArray)(JNIEnv* env, jobject k)
{
    auto filter = getGrabberFilter(env, k);
    jfloatArray array;

    switch (filter->getOutputDataType())
    {
        case CyC_STATE:
        {
            CycState meas;
            if (!filter->getData(meas))
            {
                throwRuntimeException(env,
                        fmt::format("Could not get data from filter [{}].", filter->getFilterKey().nFilterID).c_str());
            }
            else
            {
                array = env->NewFloatArray(meas.x_hat.size());
                env->SetFloatArrayRegion(array, 0, meas.x_hat.size(), meas.x_hat.data());
            }
        }
        break;
        case CyC_IMU:
        {
            CycImu meas;
            if (!filter->getData(meas))
            {
                throwRuntimeException(env,
                                      fmt::format("Could not get data from filter [{}].", filter->getFilterKey().nFilterID).c_str());
            }
            else
            {
                array = env->NewFloatArray(9U);

                float* data = new float[9];
                data[0] = meas.accX;
                data[1] = meas.accY;
                data[2] = meas.accZ;
                data[3] = meas.gyroX;
                data[4] = meas.gyroY;
                data[5] = meas.gyroZ;
                data[6] = meas.roll;
                data[7] = meas.pitch;
                data[8] = meas.yaw;

                env->SetFloatArrayRegion(array, 0, 9, data);
                delete[] data;
            }
        }
        break;
        default:
        {
            throwRuntimeException(env,
                    fmt::format("Filter [{}] does not output float arrays.", filter->getFilterKey().nFilterID).c_str());
        }
        break;
    }

    return array;
}

DEF_METHOD(jbyteArray, CDataGrabber, getBytes)(JNIEnv* env, jobject k)
{
    throwRuntimeException(env, "Method not implemented. Check jni/CDataGrabber.cpp for getBytes.");
    return nullptr;
}