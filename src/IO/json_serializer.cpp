#include "json_serializer.hpp"

using namespace nlohmann;

std::unique_ptr<CraterJsonSerializer> CraterJsonSerializer::create(Config config)
{
    return std::unique_ptr<CraterJsonSerializer>(new CraterJsonSerializer(config));
}

CraterJsonSerializer::CraterJsonSerializer(Config config): config_(std::move(config)) {}

json CraterJsonSerializer::serialize(std::shared_ptr<Crater> mainCrater,
    const std::vector<std::pair<std::shared_ptr<Crater>, double>> &similarCraters) const
{
    json output;

    // 序列化主坑
    output["main"] = serialize_single(mainCrater);

    // 序列化相似坑
    json similar_array = json::array();
    for (const auto& [crater, prob]: similarCraters)
    {
        similar_array.push_back(
        {
            {"crater", serialize_single(crater)},
            {"probability", round(prob * 100) / 100}
        });
    }
    output["similar"] = similar_array;

    return output;
}


json CraterJsonSerializer::serialize_single(std::shared_ptr<Crater> crater) const
{
    auto j = serialize_basic(crater);
    if (config_.include_neighbors)
    {
        serialize_neighbors(crater, j);
    }
    return j;
}

json CraterJsonSerializer::serialize_basic(std::shared_ptr<Crater> crater) const
{
    const auto round_val = [this](double v)
    {
        return std::round(v * std::pow(10, config_.float_precision)) / std::pow(10, config_.float_precision);
    };

    return
    {
        {"id", crater->get_id()},
        {"coordinates", crater->get_coordinates()},
        {"width", crater->get_width()},
        {"height",crater->get_height()},
        {"aspect_ratio", round_val(crater->get_aspectRatio())},
        {"area", crater->get_area()}
    };
}

void CraterJsonSerializer::serialize_neighbors(std::shared_ptr<Crater> crater, json& j) const
{
    json neighbors = json::array();
    for (const auto& neighbor: crater->get_neighbors())
    {
        neighbors.push_back(serialize_basic(neighbor));
    }
    j["neighbors"] = neighbors;
}