void draco_encode_fields(int8_t *pdata, int &num, draco::EncoderBuffer &buffer, std::vector<int> &vec_offset, std::vector<ld_data::DataType> &vec_data_type,std::vector<int> &vec_count)
{
    if(num == 0)
    {
        return ;
    } 
    draco::PointCloudBuilder builder;

    std::vector<int> vec_att_id;
    std::vector<draco::DataType> vec_draco_type;
    int struct_size = 12;
    draco::DataType data_type;
    int pos_att_id;
    // auto fields = ld_data::Field(data);
    int att_id_num = vec_offset.size();
    // std::cout << "att_id_num " << att_id_num << "\n";
    if(att_id_num < 3)
    {
        return;
    }
    builder.Start(num);
    pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);         
    vec_att_id.push_back(pos_att_id);
    vec_att_id.push_back(pos_att_id);
    vec_att_id.push_back(pos_att_id);
    for(auto i=3;i<att_id_num;i++)
    {
        int data_size = getdracotype(vec_data_type[i], data_type);
        // vec_draco_type.push_back(data_type);      
        pos_att_id = builder.AddAttribute(draco::GeometryAttribute::GENERIC, vec_count[i], data_type);            
        // std::cout << "pos_att_id: " << pos_att_id << "\n";
        vec_att_id.push_back(pos_att_id);
        struct_size += vec_count[i]*data_size;
    }
    if(struct_size == 0)
    {
        return ;
    }  
    std::cout << "struct_size: " << struct_size << "\n";
    for (int index = 0; index < num; index++) {
        // std::array<float, 3> point;
        // memcpy(&(point[0]),&(pdata[struct_size*index]),3*4);
        // builder.SetAttributeValueForPoint(vec_att_id[0], draco::PointIndex(index), &(point)[0]);
        // std::array<float, 1> intensity;
        // memcpy(&(intensity)[0],&(pdata[struct_size*index + 12]),4);
        // builder.SetAttributeValueForPoint(id1, draco::PointIndex(index), &(intensity[0]));            
        builder.SetAttributeValueForPoint(vec_att_id[0], draco::PointIndex(index), &(pdata[struct_size*index + vec_offset[0]]));            
        for(auto i=3;i<att_id_num;i++) 
        {
            builder.SetAttributeValueForPoint(vec_att_id[i], draco::PointIndex(index), &(pdata[struct_size*index + vec_offset[i]]));            
        }       
    }
    std::cout << "builder over" << "\n";
    std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);

    // encode/decode using kd-tree
    int compression_level = cl_;
    // EncoderBuffer buffer;
    draco::PointCloudKdTreeEncoder encoder;
    draco::EncoderOptions options = draco::EncoderOptions::CreateDefaultOptions();
    options.SetGlobalInt("quantization_bits", qb_);
    options.SetSpeed(10 - compression_level, 10 - compression_level);
    encoder.SetPointCloud(*pc);
    double t1 = NDKGetTime();
    MyAssert(encoder.Encode(options, &buffer).ok(), 2001);
    double t2 = NDKGetTime();
    printf("fileds KD-Tree Compression ratio: %.6f \
        \noriginal bytes: %ld \
        \ncompressioned bytes: %d \
        \nKD-tree Encoding time: %.6fs\n", \
        buffer.size()/(num * (12.0f + 4.0f)), num * (12 + 4), (int)buffer.size(), t2 - t1);
    printf("compress: %d %.6f %.6fs\n",qb_, buffer.size()/(num * (12.0f + 4.0f)),t2 - t1);
}