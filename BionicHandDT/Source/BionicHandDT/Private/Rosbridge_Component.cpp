#include "Rosbridge_Component.h"
#include "WebSocketsModule.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

void URosbridgeComponent::Connect(const FString& Url)
{
    AdvertisedTopics.Empty();

    Socket = FWebSocketsModule::Get().CreateWebSocket(Url);
    Socket->OnConnected().AddLambda([]() { UE_LOG(LogTemp, Log, TEXT("ROS connected")); });
    Socket->OnConnectionError().AddLambda([](const FString& E) { UE_LOG(LogTemp, Error, TEXT("ROS error: %s"), *E); });
    Socket->OnMessage().AddLambda([this](const FString& Msg)
        {
            TSharedPtr<FJsonObject> Obj;
            auto Reader = TJsonReaderFactory<>::Create(Msg);
            if (!FJsonSerializer::Deserialize(Reader, Obj) || !Obj.IsValid()) return;
            if (Obj->GetStringField(TEXT("op")) != TEXT("publish")) return;
            FString Out;
            auto Writer = TJsonWriterFactory<>::Create(&Out);
            FJsonSerializer::Serialize(Obj->GetObjectField(TEXT("msg")).ToSharedRef(), Writer);
            OnRosMessage.Broadcast(Obj->GetStringField(TEXT("topic")), Out);
        });
    Socket->Connect();
}

void URosbridgeComponent::Send(const FString& S)
{
    if (Socket.IsValid() && Socket->IsConnected()) Socket->Send(S);
}

void URosbridgeComponent::Subscribe(const FString& Topic, const FString& Type)
{
    Send(FString::Printf(TEXT("{\"op\":\"subscribe\",\"topic\":\"%s\",\"type\":\"%s\"}"), *Topic, *Type));
}

void URosbridgeComponent::SendTwist(float Linear, float Angular)
{
    if (!bAdvertised)
    {
        Send(TEXT("{\"op\":\"advertise\",\"topic\":\"/cmd_vel\",\"type\":\"geometry_msgs/msg/Twist\"}"));
        bAdvertised = true;
    }
    Send(FString::Printf(TEXT("{\"op\":\"publish\",\"topic\":\"/cmd_vel\",\"msg\":{\"linear\":{\"x\":%f,\"y\":0,\"z\":0},\"angular\":{\"x\":0,\"y\":0,\"z\":%f}}}"), Linear, Angular));
}

void URosbridgeComponent::PublishString(const FString& Topic, const FString& Data)
{
    if (!AdvertisedTopics.Contains(Topic))
    {
        Send(FString::Printf(TEXT("{\"op\":\"advertise\",\"topic\":\"%s\",\"type\":\"std_msgs/msg/String\"}"), *Topic));
        AdvertisedTopics.Add(Topic);
    }

    TSharedRef<FJsonObject> Msg = MakeShared<FJsonObject>();
    Msg->SetStringField(TEXT("data"), Data);

    TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("op"), TEXT("publish"));
    Root->SetStringField(TEXT("topic"), Topic);
    Root->SetObjectField(TEXT("msg"), Msg);

    FString Out;
    auto Writer = TJsonWriterFactory<>::Create(&Out);
    FJsonSerializer::Serialize(Root, Writer);
    Send(Out);
}

void URosbridgeComponent::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Socket.IsValid()) Socket->Close();
    Super::EndPlay(Reason);
}