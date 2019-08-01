#include <waypoint_planner/WaypointPolicyDataset.h>

WaypointPolicyDataset::WaypointPolicyDataset(std::string& file_names_csv)
// Load csv file with file locations and labels.
: csv(ReadCsv(file_names_csv))
{

}

// Override the get method to load custom data.
torch::data::Example<> WaypointPolicyDataset::get(size_t index)
{
//  std::string file_location = std::get<0>(csv[index]);
//  int label = std::get<1>(csv[index]);
//
//  // Load image with OpenCV.
//  cv::Mat img = cv::imread(file_location);
//
//  // Convert the image and label to a tensor.
//  torch::Tensor img_tensor = torch::from_blob(img.data, {img.rows, img.cols, 3}, torch::kByte);
//  img_tensor = img_tensor.permute({2, 0, 1}); // convert to CxHxW
//
//  torch::Tensor label_tensor = torch::full({1}, label);

  torch::Tensor img_tensor = torch::zeros(2);
  torch::Tensor label_tensor = torch::zeros(1);

  return {img_tensor.clone(), label_tensor.clone()};
}

// Override the size method to infer the size of the data set.
torch::optional<size_t> size()
{
  return csv.size();
}

// Read in the csv file and return file locations and labels as vector of tuples.
auto ReadCsv(std::string& location) -> std::vector<std::tuple<std::string /*file prefix*/, int /*label*/>> {

  std::fstream in(location, std::ios::in);
  std::string line;
  std::string name;
  std::string label;
  std::vector<std::tuple<std::string, int>> csv;

  while (getline(in, line))
  {
    std::stringstream s(line);
    getline(s, label, ',');
    getline(s, name, ',');

    csv.push_back(std::make_tuple(name, stoi(label)));
  }

  return csv;
}