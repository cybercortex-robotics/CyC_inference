// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#ifndef CyC_JNI_COMMON_H
#define CyC_JNI_COMMON_H

#include <jni.h>
#include <android/log.h>
#include <unordered_map>

#include <CyC_TYPES.h>

extern std::unordered_map<CyC_UINT, jlong> g_CcrDataTypeMap;
extern std::unordered_map<CyC_UINT, jlong> g_CcrFilterTypeMap;

#define DEF_METHOD(returntype, classname, methodname) \
extern "C" \
JNIEXPORT returntype \
JNICALL Java_ai_cybercortex_testervisioncore_1android_jni_##classname##_##methodname

template <typename JNIType>
JNIType* getPtrFromAddressField(JNIEnv* env, jobject k)
{
    jclass cls = env->GetObjectClass(k);
    jfieldID fid = env->GetFieldID(cls, "mAddress", "J");
    jlong addr = env->GetLongField(k, fid);

    return reinterpret_cast<JNIType*>(addr);
}

static void throwRuntimeException(JNIEnv* env, const char* msg)
{
    env->ThrowNew(env->FindClass("java/lang/RuntimeException"), msg);
}

#endif // CyC_JNI_COMMON_H
