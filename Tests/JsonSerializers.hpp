#pragma once

#include <string>
#include <filesystem>

class JsonInternalData;

typedef std::string String;

struct PolymorphicInfo
{
  String mName;
};

class JsonSaver
{
public:

  JsonSaver();
  ~JsonSaver();
  String ToString();

  virtual bool SerializePrimitive(char* data);
  virtual bool SerializePrimitive(bool& data);
  virtual bool SerializePrimitive(char& data);
  virtual bool SerializePrimitive(int& data);
  virtual bool SerializePrimitive(float& data);
  virtual bool SerializePrimitive(double& data);
  virtual bool SerializePrimitive(String& data);

  virtual bool BeginObject();
  virtual bool BeginObject(PolymorphicInfo& info);
  virtual bool BeginMembers(size_t& count);
  virtual bool BeginMember(const String& name);
  virtual bool BeginMember(size_t index, String& name);
  virtual bool BeginArray(size_t& count);
  virtual bool BeginArrayItem(size_t index);
  virtual bool EndObject();
  virtual bool EndMember();
  virtual bool EndMembers();
  virtual bool EndArray();
  virtual bool EndArrayItem();

  bool WriteKey(const std::string& name);
  bool WritePrimitive(bool data);
  bool WritePrimitive(char data);
  bool WritePrimitive(int data);
  bool WritePrimitive(float data);
  bool WritePrimitive(double data);
  bool WritePrimitive(const String& data);

  JsonInternalData* mData;
};

class JsonLoader
{
public:
  JsonLoader();
  ~JsonLoader();

  void Load(const String& jsonData);
  void LoadFromFile(const String& filePath);

  virtual bool SerializePrimitive(char* data);
  virtual bool SerializePrimitive(bool& data);
  virtual bool SerializePrimitive(char& data);
  virtual bool SerializePrimitive(int& data);
  virtual bool SerializePrimitive(float& data);
  virtual bool SerializePrimitive(double& data);
  virtual bool SerializePrimitive(String& data);

  virtual bool BeginObject();
  virtual bool BeginObject(PolymorphicInfo& info);
  virtual bool BeginMembers(size_t& count);
  virtual bool BeginMember(const String& name);
  virtual bool BeginMember(size_t index, String& name);
  virtual bool BeginArray(size_t& count);
  virtual bool BeginArrayItem(size_t index);
  virtual bool EndObject();
  virtual bool EndMembers();
  virtual bool EndMember();
  virtual bool EndArray();
  virtual bool EndArrayItem();

  bool ReadPrimitive(bool& data);
  bool ReadPrimitive(char& data);
  bool ReadPrimitive(int& data);
  bool ReadPrimitive(float& data);
  bool ReadPrimitive(double& data);
  bool ReadPrimitive(String& data);

  bool BeginMember();
  bool End();

  JsonInternalData* mData;
};


template <typename ArrayType, size_t ArraySize = ArrayType::Count>
bool LoadArray(JsonLoader& loader, const String& name, ArrayType& data)
{
  if(!loader.BeginMember(name))
    return false;

  size_t count;
  loader.BeginArray(count);
  for(size_t i = 0; i < count; ++i)
  {
    loader.BeginArrayItem(i);
    loader.SerializePrimitive(data[i]);
    loader.EndArrayItem();
  }
  loader.EndArray();
  loader.EndMember();
  return true;
}
template <typename DataType>
bool LoadPrimitive(JsonLoader& loader, const String& name, DataType& data)
{
  if(!loader.BeginMember(name))
    return false;

  loader.SerializePrimitive(data);
  loader.EndMember();
  return true;
}
template <typename DataType>
DataType LoadDefaultPrimitive(JsonLoader& loader, const String& name, const DataType& defaultValue)
{
  DataType result = defaultValue;
  LoadPrimitive(loader, name, result);
  return result;
}

template <typename ManagerType>
void LoadAllFilesOfExtension(ManagerType& manager, const String& searchDir, const String& extension, bool recursive = true)
{
  for(auto it : std::filesystem::directory_iterator(searchDir))
  {
    if(it.is_directory())
    {
      if(recursive)
        LoadAllFilesOfExtension(manager, it.path().string(), extension, recursive);
      continue;
    }
    
    auto filePath = it.path();
    if(filePath.extension() == extension)
      manager.LoadFromFile(filePath.string());
  }
}
