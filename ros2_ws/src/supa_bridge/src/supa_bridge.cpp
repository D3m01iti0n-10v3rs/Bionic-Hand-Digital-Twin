#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

static const std::string SUPABASE_URL = "https://ecxiewcuwglmbxfvpdfr.supabase.co/rest/v1/telemetry";
static const std::string SUPABASE_KEY = "sb_publishable_TzRpHuyStf5XNLAFwwVlsw_h36Ku_se";

class SupaBridge : public rclcpp::Node{
public:
  SupaBridge() : Node("supa_bridge"){
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, [this](std_msgs::msg::String::SharedPtr m) { post(m->data); });
  }

private:
  void post(const std::string &v){
    nlohmann::json j = {{"topic", "chatter"}, {"data", {{"v", v}}}};
    std::string body = j.dump();
    CURL *c = curl_easy_init();
    curl_slist *h = nullptr;

    h = curl_slist_append(h, ("apikey: " + SUPABASE_KEY).c_str());
    h = curl_slist_append(h, "Content-Type: application/json");

    curl_easy_setopt(c, CURLOPT_URL, SUPABASE_URL.c_str());
    curl_easy_setopt(c, CURLOPT_HTTPHEADER, h);
    curl_easy_setopt(c, CURLOPT_POSTFIELDS, body.c_str());

    CURLcode rc = curl_easy_perform(c);

    long code = 0;
    curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &code);
    if (rc != CURLE_OK || code >= 300)
      RCLCPP_ERROR(get_logger(), "POST failed: curl=%d http=%ld", rc, code);

    curl_slist_free_all(h);
    curl_easy_cleanup(c);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupaBridge>());
  rclcpp::shutdown();
}