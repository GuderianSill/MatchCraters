#ifndef SERIALIZER_INTERFACE_HPP
#define SERIALIZER_INTERFACE_HPP

#include <vector>
#include <crater.hpp>
#include <memory>
#include <nlohmann/json.hpp>

class ICraterSerializer
{
public:
    using json = nlohmann::json;

    virtual ~ICraterSerializer() = default;

    virtual json serialize(std::shared_ptr<Crater> mainCrater, const std::vector<std::pair<std::shared_ptr<Crater>, double>>& similarCrater) const = 0;

    virtual json serialize_single(std::shared_ptr<Crater> crater) const = 0;
};


#endif