// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDNNCONFIG_H_
#define CDNNCONFIG_H_

#include <map>
#include <vector>
#include "CyC_TYPES.h"
#include "os/CConversions.h"
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)

class CDnnConfig
{
public:
    // Types of transformations
    enum TransformType
    {
        Transform_UNDEFINED = 0,
        Transform_RGB2BGR = 1,
        Transform_RGB2GRAY = 2,
        Transform_GRAY2RGB = 3,
        Transform_NORMALIZATION = 4
    };

    // Transforms applied to a DNN's I/O data
    struct Transform
    {
        std::string                      name;
        TransformType                   type;
        std::vector<Eigen::VectorXf>    paramenters;
    };
    typedef std::vector<Transform> Transforms;

    // Config structure for a Deep Neural Network
    struct Common
    {
        std::string    device;
        bool      is_training;
        CyC_UINT      epochs;
        float     learning_rate;
        float     momentum;
        CyC_UINT      batch_size;
        bool      shuffle;
        CyC_UINT      num_workers;
        float     train_split;
        std::string    optimizer;
        bool      tensorboard;
        bool      plot_architecture;
        bool      view_predictions;
        std::string    ckpts_dir;
        CyC_UINT      ckpt_freq;
        bool      load_last_ckpt;
        bool      onnx_export;
        CyC_UINT      onnx_opset_version;
        std::string    onnx_model_file;
        CyC_UINT      load_pretrained_weights;
        std::string    pretrained_weights;

        std::vector<CyC_DATA_TYPE>          input_data;
        std::vector<std::vector<CyC_INT>>   input_shape;
        std::vector<CyC_DATA_TYPE>          output_data;
        std::vector<std::vector<CyC_INT>>   output_shape;

        std::vector<Transforms>             input_data_transforms;
    };

public:
    CDnnConfig(const std::string& _dnn_config_file);
    virtual ~CDnnConfig();
    
    bool isInitialized()              { return m_bIsInitialized; };
    CDnnConfig::Common* getDnnConfig()    { return &m_Common; };

    TransformType String2TransformationType(const std::string& sTransformationType);

//private:
    bool    m_bIsInitialized;
    Common  m_Common;
};

#endif /* CDNNCONFIG_H_ */
