#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "IWebSocket.h"
#include "Rosbridge_Component.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnRosMessage, const FString&, Topic, const FString&, MsgJson); // create event

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent)) // register the class with unreal
class BIONICHANDDT_API URosbridgeComponent : public UActorComponent{
    GENERATED_BODY()

    public:
        UFUNCTION(BlueprintCallable, Category = "Ros") void Connect(const FString& Url);
        UFUNCTION(BlueprintCallable, Category = "Ros") void Subscribe(const FString& Topic, const FString& Type);

        UFUNCTION(BlueprintCallable, Category = "Ros") void SendTwist(float Linear, float Angular);
        UFUNCTION(BlueprintCallable, Category = "Ros") void PublishString(const FString& Topic, const FString& Data);

        UPROPERTY(BlueprintAssignable, Category = "Ros") FOnRosMessage OnRosMessage;

    protected:
        virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    private:
        TSharedPtr<IWebSocket> Socket;
        bool bAdvertised = false;
        TSet<FString> AdvertisedTopics;
        void Send(const FString& S);
};