#ifndef JSON_SERIALIZER_HPP
#define JSON_SERIALIZER_HPP
#include "serializer_interface.hpp"

class CraterJsonSerializer: public ICraterSerializer
{
public:
    struct Config
    {
        bool include_neighbors = false;
        int float_precision = 3;
    };

    static std::unique_ptr<CraterJsonSerializer> create(Config config);

    // 主接口
    json serialize(std::shared_ptr<Crater> mainCrater,
                const std::vector<std::pair<std::shared_ptr<Crater>, double>>& similarCraters) const override;
    
    json serialize_single(std::shared_ptr<Crater> crater) const override;

protected:
    explicit CraterJsonSerializer(Config config);

private:
    json serialize_basic(std::shared_ptr<Crater> crater) const;
    virtual void serialize_neighbors(std::shared_ptr<Crater> crater, json& j) const;

    const Config config_;
};


#endif