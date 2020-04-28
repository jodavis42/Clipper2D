//#include "pch.h"

#include "JsonSerializers.hpp"
//#include "StaticTypeId.hpp"
//#include "SerializationHelpers.hpp"

#include "RapidJson\rapidjson.h"
#include "RapidJson\document.h"
#include "RapidJson\prettywriter.h"
#include "RapidJson\writer.h"
#include "RapidJson\stringbuffer.h"
#include <stack>
#include <fstream>
#include <sstream>

class JsonInternalData
{
public:
  JsonInternalData() : mWriter(mBuffer)
  {
  }
  rapidjson::Document mDocument;
  rapidjson::StringBuffer mBuffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> mWriter;
  rapidjson::Writer<rapidjson::StringBuffer> mReader;
  std::stack<rapidjson::Value*> mStack;
};

JsonSaver::JsonSaver()
{
  mData = new JsonInternalData();
}

JsonSaver::~JsonSaver()
{
  delete mData;
}

std::string JsonSaver::ToString()
{
  return mData->mBuffer.GetString();
}

bool JsonSaver::SerializePrimitive(char* data)
{
  __debugbreak();
  return false;
}

bool JsonSaver::SerializePrimitive(bool& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::SerializePrimitive(char& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::SerializePrimitive(int& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::SerializePrimitive(float& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::SerializePrimitive(double& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::SerializePrimitive(String& data)
{
  return WritePrimitive(data);
}

bool JsonSaver::BeginObject()
{
  return mData->mWriter.StartObject();
}

bool JsonSaver::BeginObject(PolymorphicInfo& info)
{
  BeginObject();
  WriteKey("Typename");
  WritePrimitive(info.mName);
  WriteKey("Value");
  return true;
}

bool JsonSaver::BeginMembers(size_t& count)
{
  return BeginObject();
}

bool JsonSaver::BeginMember(const String& name)
{
  return WriteKey(name);
}

bool JsonSaver::BeginMember(size_t index, String& name)
{
  return WriteKey(name);
}

bool JsonSaver::BeginArray(size_t& count)
{
  return mData->mWriter.StartArray();
}

bool JsonSaver::BeginArrayItem(size_t index)
{
  //return BeginObject();
  return true;
}

bool JsonSaver::EndObject()
{
  return mData->mWriter.EndObject();
}

bool JsonSaver::EndMembers()
{
  return EndObject();
}

bool JsonSaver::EndMember()
{
  return true;
}

bool JsonSaver::EndArray()
{
  return mData->mWriter.EndArray();
}

bool JsonSaver::EndArrayItem()
{
  //return EndObject();
  return true;
}

bool JsonSaver::WriteKey(const String& name)
{
  return mData->mWriter.Key(name.c_str());
}

bool JsonSaver::WritePrimitive(bool data)
{
  return mData->mWriter.Bool(data);
}

bool JsonSaver::WritePrimitive(char data)
{
  return mData->mWriter.Int(data);
}

bool JsonSaver::WritePrimitive(int data)
{
  return mData->mWriter.Int(data);
}

bool JsonSaver::WritePrimitive(float data)
{
  return mData->mWriter.Double(data);
}

bool JsonSaver::WritePrimitive(double data)
{
  return mData->mWriter.Double(data);
}

bool JsonSaver::WritePrimitive(const String& data)
{
  return mData->mWriter.String(data.c_str());
}

JsonLoader::JsonLoader()
{
  mData = new JsonInternalData();
}

JsonLoader::~JsonLoader()
{
  delete mData;
}

void JsonLoader::Load(const String& jsonData)
{
  mData->mDocument.Parse(jsonData.c_str());
  mData->mStack.push(&mData->mDocument);
}

void JsonLoader::LoadFromFile(const String& filePath)
{
  std::ifstream stream;
  stream.open(filePath, std::ifstream::in);

  std::stringstream strStream;
  strStream << stream.rdbuf();

  Load(strStream.str());
}

bool JsonLoader::SerializePrimitive(char* data)
{
  __debugbreak();
  return false;
}

bool JsonLoader::SerializePrimitive(bool& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::SerializePrimitive(char& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::SerializePrimitive(int& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::SerializePrimitive(float& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::SerializePrimitive(double& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::SerializePrimitive(String& data)
{
  return ReadPrimitive(data);
}

bool JsonLoader::BeginObject()
{
  return BeginMember();
}

bool JsonLoader::BeginObject(PolymorphicInfo& info)
{
  rapidjson::Value* node = mData->mStack.top();

  auto typenameIt = node->FindMember("Typename");
  rapidjson::Value& typenameNode = typenameIt->value;
  info.mName = typenameNode.GetString();

  auto valueIt = node->FindMember("Value");
  rapidjson::Value& valueNode = valueIt->value;
  mData->mStack.push(&valueNode);

  return true;
}

bool JsonLoader::BeginMembers(size_t& count)
{
  rapidjson::Value* node = mData->mStack.top();
  count = node->MemberCount();
  return true;
}

bool JsonLoader::BeginMember(const String& name)
{
  rapidjson::Value* node = mData->mStack.top();
  auto it = node->FindMember(name.c_str());
  if(it == node->MemberEnd())
    return false;

  rapidjson::Value& valueNode = it->value;
  mData->mStack.push(&valueNode);
  return true;
}

bool JsonLoader::BeginMember(size_t index, String& name)
{
  rapidjson::Value* node = mData->mStack.top();
  if(index >= node->MemberCount())
    return false;
  auto it = node->MemberBegin();
  it += index;

  name = it->name.GetString();

  rapidjson::Value& valueNode = it->value;
  mData->mStack.push(&valueNode);
  return true;
}

bool JsonLoader::BeginArray(size_t& count)
{
  rapidjson::Value* arrayNode = mData->mStack.top();
  auto array = arrayNode->GetArray();
  count = array.Size();
  return true;
}

bool JsonLoader::BeginArrayItem(size_t index)
{
  rapidjson::Value* arrayNode = mData->mStack.top();
  auto array = arrayNode->GetArray();
  if(index >= array.Size())
    return false;

  rapidjson::Value& arrayItemNode = array[(int)index];
  mData->mStack.push(&arrayItemNode);
  return true;
}

bool JsonLoader::EndObject()
{
  return End();
}

bool JsonLoader::EndMembers()
{
  return End();
}

bool JsonLoader::EndMember()
{
  return End();
}

bool JsonLoader::EndArrayItem()
{
  return EndObject();
}

bool JsonLoader::EndArray()
{
  return true;
}

bool JsonLoader::ReadPrimitive(bool& data)
{
  rapidjson::Value* node = mData->mStack.top();
  
  if(node->IsBool())
  {
    data = node->GetBool();
    return true;
  }

  if(node->IsInt())
  {
    data = node->GetInt() != 0;
    return true;
  }

  return false;
}

bool JsonLoader::ReadPrimitive(char& data)
{
  rapidjson::Value* node = mData->mStack.top();
  if(node->IsBool())
  {
    data = node->GetBool() ? 1 : 0;
    return true;
  }

  if(node->IsInt())
  {
    data = node->GetInt();
    return true;
  }

  return false;
}

bool JsonLoader::ReadPrimitive(int& data)
{
  rapidjson::Value* node = mData->mStack.top();
  if(node->IsBool())
  {
    data = node->GetBool() ? 1 : 0;
    return true;
  }

  if(node->IsInt())
  {
    data = node->GetInt();
    return true;
  }
  return false;
}

bool JsonLoader::ReadPrimitive(float& data)
{
  rapidjson::Value* node = mData->mStack.top();
  if(node->IsBool())
  {
    data = node->GetBool() ? 1.0f : 0.0f;
    return true;
  }

  if(node->IsInt())
  {
    data = static_cast<float>(node->GetInt());
    return true;
  }
  
  if(node->IsDouble())
  {
    data = static_cast<float>(node->GetDouble());
    return true;
  }

  return false;
}

bool JsonLoader::ReadPrimitive(double& data)
{
  rapidjson::Value* node = mData->mStack.top();
  if(node->IsBool())
  {
    data = node->GetBool() ? 1.0 : 0.0;
    return true;
  }

  if(node->IsInt())
  {
    data = static_cast<double>(node->GetInt());
    return true;
  }

  if(node->IsDouble())
  {
    data = node->GetDouble();
    return true;
  }

  return false;
}

bool JsonLoader::ReadPrimitive(String& data)
{
  rapidjson::Value* node = mData->mStack.top();
  if(!node->IsString())
    return false;

  data = node->GetString();
  return true;
}

bool JsonLoader::BeginMember()
{
  rapidjson::Value* node = mData->mStack.top();

  auto it = node->MemberBegin();
  if(it == node->MemberEnd())
    return false;

  rapidjson::Value& valueNode = it->value;
  mData->mStack.push(&valueNode);
  return true;
}

bool JsonLoader::End()
{
  if(mData->mStack.empty())
    return false;
  mData->mStack.pop();
  return true;
}
