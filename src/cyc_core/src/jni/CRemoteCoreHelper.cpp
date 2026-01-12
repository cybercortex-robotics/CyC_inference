// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Mihai Zaha

#include "common.h"

#include "../CCycCore.h"
#include "../communication/CCommunication.h"
#include <msgpack.hpp>

CCycCore* getRemoteCore(JNIEnv* env, jobject k)
{
    jclass cls = env->GetObjectClass(k);
    jfieldID fid = env->GetFieldID(cls, "mCore", "Lai/cybercortex/testervisioncore_android/jni/CCycCore;");
    jobject core = env->GetObjectField(k, fid);

    return getPtrFromAddressField<CCycCore>(env, core);
}

void sendCommand(CCycCore* core, CyC_INT remoteCoreId, ECommand cmd, CyC_INT remoteFilterId = 0)
{
    msgpack::sbuffer buf;
    msgpack::packer packer(buf);

    packer.pack(core->getVisionCoreID());
    packer.pack((CyC_INT)0);
    packer.pack((CyC_INT)CCommunication::EInternalTypes::CyC_COMMAND);
    packer.pack((CyC_INT)cmd);
    packer.pack((CyC_ULONG)remoteCoreId);
    packer.pack((CyC_INT)remoteFilterId);

    core->getCommunication()->sendPacket(remoteCoreId, buf);
}

DEF_METHOD(void, CRemoteCoreHelper, enableAllFilters)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000; // TODO
    sendCommand(core, remoteCoreId, ECommand::ENABLE_FILTERS);
}

DEF_METHOD(void, CRemoteCoreHelper, startAllFilters)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    sendCommand(core, remoteCoreId, ECommand::START_FILTERS);
}

DEF_METHOD(void, CRemoteCoreHelper, stopAllFilters)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    sendCommand(core, remoteCoreId, ECommand::STOP_FILTERS);
}

DEF_METHOD(void, CRemoteCoreHelper, disableAllFilters)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    sendCommand(core, remoteCoreId, ECommand::DISABLE_FILTERS);
}

DEF_METHOD(void, CRemoteCoreHelper, requestCoreInfo)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    sendCommand(core, remoteCoreId, ECommand::REQUEST_INFO);
}

DEF_METHOD(void, CRemoteCoreHelper, startStopFilter)(JNIEnv* env, jobject k, jint filterId)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    sendCommand(core, remoteCoreId, ECommand::START_STOP_FILTER, filterId);
}

DEF_METHOD(jobjectArray, CRemoteCoreHelper, getFilters)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return nullptr;

    const auto& remoteFiltersInfo = core->getCommunication()->getRemoteFiltersInfo();
    if (remoteFiltersInfo.empty())
    {
        return nullptr;
    }

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    const auto& filtersInfo = remoteFiltersInfo.at(remoteCoreId);

    jclass javaRemoteCcrFilterClass = env->FindClass("ai/cybercortex/testervisioncore_android/jni/CRemoteCcrFilter");
    jmethodID javaRemoteFilterConstructor = env->GetMethodID(javaRemoteCcrFilterClass, "<init>","(ILjava/lang/String;ZZ)V");

    jobjectArray filterArray = env->NewObjectArray(filtersInfo.size(), javaRemoteCcrFilterClass, nullptr);
    for (size_t i = 0; i < filtersInfo.size(); ++i)
    {
        const auto& remoteFilter = filtersInfo[i];
        env->SetObjectArrayElement(
                filterArray, i,
                env->NewObject(
                        javaRemoteCcrFilterClass,
                        javaRemoteFilterConstructor,
                        static_cast<jint>(remoteFilter.filterID),
                        env->NewStringUTF(remoteFilter.filterName.c_str()),
                        static_cast<jboolean>(remoteFilter.enabled),
                        static_cast<jboolean>(remoteFilter.running)));
    }

    return filterArray;
}

DEF_METHOD(jboolean, CRemoteCoreHelper, isConnectionOpened)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return false;

    return core->getCommunication()->isConnectionOpened(core->getVisionCoreID() - 1000);
}

DEF_METHOD(jlong, CRemoteCoreHelper, getPassedMillis)(JNIEnv* env, jobject k)
{
    auto core = getRemoteCore(env, k);
    if (core == nullptr)
        return 0;

    auto remoteCoreId = core->getVisionCoreID() - 1000;
    const auto& lastReceivedTimestamps = core->getCommunication()->getLastReceivedRemoteInfo();
    if (lastReceivedTimestamps.empty())
    {
        return 0;
    }

    return static_cast<jlong>(CTimer::now() - lastReceivedTimestamps.at(remoteCoreId));
}