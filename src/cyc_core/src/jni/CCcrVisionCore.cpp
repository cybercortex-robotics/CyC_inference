// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#include "common.h"
#include <string>

#include "../CCycCore.h"

CCycCore* getCore(JNIEnv* env, jobject k)
{
    return getPtrFromAddressField<CCycCore>(env, k);
}

DEF_METHOD(jboolean, CCycCore, init)(JNIEnv* env, jobject k, jstring filename)
{
    jclass cls = env->GetObjectClass(k);
    jfieldID fid = env->GetFieldID(cls, "mAddress", "J");

    std::string szFilename = env->GetStringUTFChars(filename, nullptr);
    __android_log_print(ANDROID_LOG_DEBUG, "CCycCore", "Creating CycCore with conf file: %s", szFilename.c_str());

    auto core = new CCycCore();

    std::vector<std::string> szNetworkConfFilenames;
    if (core->init(szFilename, szNetworkConfFilenames))
    {
        env->SetLongField(k, fid, reinterpret_cast<jlong>(core));
        return 1;
    }
    else
    {
        delete core;
        __android_log_print(ANDROID_LOG_ERROR, "CCycCore", "Failed to initialize core.");
    }

    return 0;
}

DEF_METHOD(void, CCycCore, deinit)(JNIEnv* env, jobject k)
{
    auto core = getCore(env, k);
    if (core != nullptr)
    {
        core->stopAllFilters();
        core->disableAllFilters();
    }

    delete core;
}

DEF_METHOD(jint, CCycCore, getVisionCoreID)(JNIEnv* env, jobject k)
{
    return getCore(env, k)->getVisionCoreID();
}

DEF_METHOD(jstring, CCycCore, getNetworkIP)(JNIEnv* env, jobject k)
{
    // TODO
    //return env->NewStringUTF(CConfigParameters::instance().getNetworkConfiguration().ip.c_str());
    return env->NewStringUTF("");
}

DEF_METHOD(jint, CCycCore, getNetworkPort)(JNIEnv* env, jobject k)
{
    // TODO
    //return CConfigParameters::instance().getNetworkConfiguration().port;
    return 0;
}

DEF_METHOD(jstring, CCycCore, getReplayDBPath)(JNIEnv* env, jobject k)
{
    return env->NewStringUTF(getCore(env, k)->getReplayDBPath().c_str());
}

DEF_METHOD(void, CCycCore, enableAllFilters)(JNIEnv* env, jobject k)
{
    getCore(env, k)->enableAllFilters();
}

DEF_METHOD(void, CCycCore, startAllFilters)(JNIEnv* env, jobject k)
{
    getCore(env, k)->startAllFilters();
}

DEF_METHOD(void, CCycCore, disableAllFilters)(JNIEnv* env, jobject k)
{
    getCore(env, k)->enableAllFilters();
}

DEF_METHOD(void, CCycCore, stopAllFilters)(JNIEnv* env, jobject k)
{
    getCore(env, k)->stopAllFilters();
}

DEF_METHOD(jobjectArray, CCycCore, getFilters)(JNIEnv* env, jobject k)
{
    auto core = getCore(env, k);

    jclass javaBaseCcrFilterClass = env->FindClass("ai/cybercortex/testervisioncore_android/jni/CCycFilterBase");
    jmethodID javaBaseCcrFilterConstructor = env->GetMethodID(javaBaseCcrFilterClass, "<init>", "(J)V");

    jobjectArray filterArray = env->NewObjectArray(core->getDatablock().size(), javaBaseCcrFilterClass, nullptr);

    for (size_t i = 0; i < core->getDatablock().size(); ++i)
    {
        const auto& info = core->getDatablock()[i];

        CCycFilterBase* pFilter = nullptr;
        if (core->readFilter(info.Key, pFilter))
        {
            env->SetObjectArrayElement(
                    filterArray, i,
                    env->NewObject(
                            javaBaseCcrFilterClass,
                            javaBaseCcrFilterConstructor,
                            reinterpret_cast<jlong>(pFilter)));
        }
    }

    return filterArray;
}
