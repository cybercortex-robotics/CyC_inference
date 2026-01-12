// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#include "common.h"
#include <CCycFilterBase.h>

CCycFilterBase* getFilter(JNIEnv* env, jobject k)
{
    return getPtrFromAddressField<CCycFilterBase>(env, k);
}

DEF_METHOD(jint, CCycFilterBase, getFilterID)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getFilterKey().nFilterID;
}

DEF_METHOD(jint, CCycFilterBase, getFilterTypeInt)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getFilterType();
}

DEF_METHOD(jstring, CCycFilterBase, getFilterName)(JNIEnv* env, jobject k)
{
    return env->NewStringUTF(getFilter(env, k)->getFilterName().c_str());
}

DEF_METHOD(jboolean, CCycFilterBase, isPublishable)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isPublishable();
}

DEF_METHOD(jboolean, CCycFilterBase, isNetworkFilter)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isNetworkFilter();
}

DEF_METHOD(jboolean, CCycFilterBase, isReplayFilter)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isReplayFilter();
}

DEF_METHOD(jlong, CCycFilterBase, getSamplingTime)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getSamplingTime();
}

DEF_METHOD(jlong, CCycFilterBase, getTimestampStart)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getTimestampStart();
}

DEF_METHOD(jlong, CCycFilterBase, getTimestampStop)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getTimestampStop();
}

DEF_METHOD(jlong, CCycFilterBase, getOutputDataTypeInt)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->getOutputDataType();
}

DEF_METHOD(jobjectArray, CCycFilterBase, getInputSources)(JNIEnv* env, jobject k)
{
    auto filter = getFilter(env, k);

    jclass javaCycInputSourceClass = env->FindClass("ai/cybercortex/testervisioncore_android/jni/CycInputSource");
    jmethodID javaInputSourceConstructor = env->GetMethodID(javaCycInputSourceClass, "<init>", "()V");

    jclass javaBaseCcrFilterClass = env->FindClass("ai/cybercortexai/cybercortex/testervisioncore_android/jni/CCycFilterBase");
    jmethodID javaBaseCcrFilterConstructor = env->GetMethodID(javaBaseCcrFilterClass, "<init>", "(J)V");

    jfieldID javaISCoreID = env->GetFieldID(javaCycInputSourceClass, "coreID", "I");
    jfieldID javaISFilterID = env->GetFieldID(javaCycInputSourceClass, "filterID", "I");
    jfieldID javaISFilter = env->GetFieldID(javaCycInputSourceClass, "filter", "Lai/cybercortex/testervisioncore_android/jni/CCycFilterBase;");
    jfieldID javaISTimestampRead = env->GetFieldID(javaCycInputSourceClass, "timestampRead", "J");

    jobjectArray isArray = env->NewObjectArray(filter->getInputSources().size(), javaCycInputSourceClass, NULL);

    for (size_t i = 0; i < filter->getInputSources().size(); ++i)
    {
        const auto& inputSource = filter->getInputSources()[i];
        jobject javaInputSource = env->NewObject(javaCycInputSourceClass, javaInputSourceConstructor);

        jobject javaFilter = (inputSource.pCycFilter == nullptr)
                ? nullptr
                : env->NewObject(javaBaseCcrFilterClass,
                        javaBaseCcrFilterConstructor,
                        reinterpret_cast<jlong>(inputSource.pCycFilter));

        env->SetIntField(javaInputSource, javaISCoreID, static_cast<jint>(inputSource.SourceKey.nCoreID));
        env->SetIntField(javaInputSource, javaISFilterID, static_cast<jint>(inputSource.SourceKey.nFilterID));
        env->SetObjectField(javaInputSource, javaISFilter, javaFilter);
        env->SetLongField(javaInputSource, javaISTimestampRead, static_cast<jlong>(inputSource.tTimestampRead));

        env->SetObjectArrayElement(isArray, i, javaInputSource);
    }

    return isArray;
}

DEF_METHOD(jboolean, CCycFilterBase, enable)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->enable();
}

DEF_METHOD(jboolean, CCycFilterBase, disable)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->disable();
}

DEF_METHOD(jboolean, CCycFilterBase, start)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->start();
}

DEF_METHOD(jboolean, CCycFilterBase, stop)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->stop();
}

DEF_METHOD(jboolean, CCycFilterBase, isEnabled)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isEnabled();
}

DEF_METHOD(jboolean, CCycFilterBase, isRunning)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isRunning();
}

DEF_METHOD(jboolean, CCycFilterBase, isProcessing)(JNIEnv* env, jobject k)
{
    return getFilter(env, k)->isProcessing();
}

DEF_METHOD(void, CCycFilterBase, updateData)(JNIEnv* env, jobject k, jint d, jlong ts_start, jlong ts_stop, jlong sampling_time)
{
    throwRuntimeException(env, "Method not implemented. Check CCycFilterBase.cpp for updateData");
}