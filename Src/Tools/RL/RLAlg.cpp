/*
  File to load in and handle inference for policies
*/

#include "RLAlg.h"
#include <iostream>
#include <cassert>
#include <onnxruntime_cxx_api.h> 

// Default constructor implementation
Policy::Policy() : session(nullptr) {}

Policy::~Policy() {
    delete this->session;
}

void Policy::init(std::string policy_path) {
    std::string action_policy_path = policy_path;
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ModelSession");
    this->session = new Ort::Session(env, action_policy_path.c_str(), Ort::SessionOptions{});
}

std::vector<float> Policy::inference(std::vector<float> observation_input) {

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // create input tensor object from data values
  std::vector<int64_t> input_shape = {1, static_cast<int64_t>(observation_input.size())};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, observation_input.data(), observation_input.size(), input_shape.data(), input_shape.size());

  // score model & input tensor, get back output tensor
  std::vector<const char*> input_node_names = {"input"};
  std::vector<const char*> output_node_names = {"output"};

  // Now input_node_names contains the names of all input nodes
  std::vector<Ort::Value> output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);

  // Assuming output_tensors[0] is of type Ort::Value which represents a tensor
  std::vector<int64_t> tensor_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
  size_t total_elements = std::accumulate(tensor_shape.begin(), tensor_shape.end(), 1, std::multiplies<int64_t>());

  float* floatarr = output_tensors[0].GetTensorMutableData<float>();
  std::vector<float> action_output_vector(floatarr, floatarr + total_elements);

  return action_output_vector;
}