#ifndef RLALG_H
#define RLALG_H

#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>

class Policy {
    public:
        Policy();
        ~Policy();
        void init(std::string policy_path);
        std::vector<float> inference(std::vector<float> observation_input);

    private:
        Ort::Session* session;
};

#endif // RLALG_H