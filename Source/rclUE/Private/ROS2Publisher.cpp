// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "ROS2Publisher.h"

#include <Engine/World.h>
#include <TimerManager.h>

UROS2Publisher* UROS2Publisher::CreateLoopPublisherWithClass(UObject* InOwner,
                                                             const FString& InTopicName,
                                                             const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                                             const float InPubFrequency)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    publisher->TopicName = InTopicName;
    publisher->PublicationFrequencyHz = InPubFrequency;
    publisher->SetDefaultDelegates();
    return publisher;
}

UROS2Publisher* UROS2Publisher::CreateLoopPublisher(UObject* InOwner,
                                                    const FString& InTopicName,
                                                    const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                                    const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                                    float InPubFrequency,
                                                    const FTopicCallback& InUpdateDelegate,
                                                    const UROS2QoS InQoS)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    publisher->MsgClass = InMsgClass;
    publisher->TopicName = InTopicName;
    publisher->PublicationFrequencyHz = InPubFrequency;
    publisher->QoS = InQoS;
    publisher->SetDelegates(InUpdateDelegate);
    return publisher;
}

UROS2Publisher* UROS2Publisher::CreatePublisherWithClass(UObject* InOwner,
                                                         const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                                         const FString& InTopicName)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    if (!InTopicName.IsEmpty())
    {
        publisher->TopicName = InTopicName;
    }
    return publisher;
}

UROS2Publisher* UROS2Publisher::CreatePublisher(UObject* InOwner,
                                                const FString& InTopicName,
                                                const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                                const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                                float InPubFrequency,
                                                const UROS2QoS InQoS)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    publisher->MsgClass = InMsgClass;
    publisher->TopicName = InTopicName;
    publisher->PublicationFrequencyHz = InPubFrequency;
    publisher->QoS = InQoS;
    publisher->SetDefaultDelegates();
    return publisher;
}

UROS2Publisher* UROS2Publisher::CreatePublisher(UObject* InOwner,
                                                const FString& InTopicName,
                                                const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                                const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                                float InPubFrequency,
                                                const rmw_qos_profile_t& InCustomQoS)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    publisher->MsgClass = InMsgClass;
    publisher->TopicName = InTopicName;
    publisher->PublicationFrequencyHz = InPubFrequency;
    publisher->QoSProfile = InCustomQoS;
    // When a custom QoS profile is provided, mark the enum QoS as unknown to
    // make it explicit that the standard QoS setting is not used.
    publisher->QoS = UROS2QoS::UnknownQoS;
    publisher->SetDefaultDelegates();
    return publisher;
}

void UROS2Publisher::InitializeTopicComponent()
{
    TimerManager = NewObject<URRTimerManager>(
        this, URRTimerManager::StaticClass(), FName(*FString::Printf(TEXT("%s_TimerManager"), *GetName())));
    const rosidl_message_type_support_t* msg_type_support = TopicMessage->GetTypeSupport();

    RclPublisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

    pub_opt.qos = GetEffectiveQoS();

    RCSOFTCHECK(rcl_publisher_init(&RclPublisher, OwnerNode->GetNode(), msg_type_support, TCHAR_TO_UTF8(*TopicName), &pub_opt));

    StartPublishTimer();

    State = UROS2State::Initialized;
}

void UROS2Publisher::StopPublishTimer()
{
    if (TimerManager != nullptr)
    {
        TimerManager->StopTimer();
    }
}

void UROS2Publisher::StartPublishTimer()
{
    if (PublicationFrequencyHz > 0 && TimerManager != nullptr)
    {
        TimerManager->LogInfo = TopicName;
        FTimerDelegate TimerDelegate = FTimerDelegate::CreateUObject(this, &UROS2Publisher::UpdateAndPublish);
        TimerManager->SetTimer(TimerDelegate, 1.f / PublicationFrequencyHz);
    }
}

void UROS2Publisher::Destroy()
{
    Super::Destroy();
    bool res = true;
    IS_ROS2NODE_INITED(OwnerNode, GetName(), res);
    if (res)
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogROS2Topic, Log, TEXT("Publisher Destroy - rcl_publisher_fini "));
        RCSOFTCHECK(rcl_publisher_fini(&RclPublisher, OwnerNode->GetNode()));
    }
    StopPublishTimer();
    UpdateDelegate.Unbind();
}

void UROS2Publisher::UpdateAndPublish()
{
    bool res = true;
    IS_TOPIC_INITED(OwnerNode, GetName(), res);
    if (!res)
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogROS2Topic, Error, TEXT("Publisher is not initialized yet "));
        return;
    }

    UpdateDelegate.ExecuteIfBound(TopicMessage);

    Publish();
}

void UROS2Publisher::Publish()
{
    PublishChecked();
}

bool UROS2Publisher::PublishChecked()
{
    bool res = true;
    IS_TOPIC_INITED(OwnerNode, GetName(), res);
    if (!res)
    {
        return false;
    }

    if (!rcl_publisher_is_valid(&RclPublisher))
    {
        UE_LOG_WITH_INFO_NAMED(
            LogROS2Topic,
            Error,
            TEXT("ROS publisher handle is invalid for topic '%s'"),
            *TopicName);
        return false;
    }

    const rcl_ret_t Result =
        rcl_publish(&RclPublisher, TopicMessage->Get(), nullptr);
    if (Result != RCL_RET_OK)
    {
        UE_LOG_WITH_INFO_NAMED(
            LogROS2Topic,
            Error,
            TEXT("rcl_publish failed for topic '%s': status=%d"),
            *TopicName,
            static_cast<int32>(Result));
        rcutils_reset_error();
        return false;
    }

    return true;
}

bool UROS2Publisher::IsReady() const
{
    return State == UROS2State::Initialized &&
        IsValid(TopicMessage) &&
        rcl_publisher_is_valid(&RclPublisher);
}

void UROS2Publisher::SetDelegates(const FTopicCallback& InUpdateDelegate)
{
    if (!InUpdateDelegate.IsBound())
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogROS2Topic, Warning, TEXT("UpdateDelegate is not set - is this on purpose? "));
    }
    UpdateDelegate.Unbind();
    UpdateDelegate = InUpdateDelegate;
}

void UROS2Publisher::SetDefaultDelegates()
{
    UpdateDelegate.Unbind();
    UpdateDelegate.BindDynamic(this, &UROS2Publisher::UpdateMessage);
}
