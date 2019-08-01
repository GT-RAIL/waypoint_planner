#pragma once

//#define _GLIBCXX_USE_CXX11_ABI 0

#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <tuple>
#include <torch/torch.h>

class WaypointPolicyDataset : public torch::data::Dataset<WaypointPolicyDataset>
{
private:
    std::vector<std::tuple<std::string /*file location*/, int64_t /*label*/>> csv;

public:
    explicit WaypointPolicyDataset(std::string& file_names_csv);

//    ~WaypointPolicyDataset() {};

    // Override the get method to load custom data.
    torch::data::Example<> get(size_t index) override;

    // Override the size method to infer the size of the data set.
    torch::optional<size_t> size() const override;
};

auto ReadCsv(std::string& location) -> std::vector<std::tuple<std::string /*file prefix*/, int /*label*/>>;
