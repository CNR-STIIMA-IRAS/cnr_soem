#ifndef COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC_MSGS
#define COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC_MSGS

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>

namespace coe_master
{

/// Structure to hold information about a single stock.
struct get_sdo_t
{
  struct Request
  {
    std::string desc;
    std::string module_id;
    uint16_t timeout_ms;

    uint16_t index;
    uint8_t subindex;

    enum sdotype_t : uint8_t
    {
      TYPE_U8 = 0,
      TYPE_U16 = 1,
      TYPE_U32 = 2,
      TYPE_U64 = 3,
      TYPE_I8 = 4,
      TYPE_I16 = 5,
      TYPE_I32 = 6,
      TYPE_I64 = 7
    } sdotype;

  } req;

  struct Response
  {
    bool success;
    std::array<uint8_t, 8> value;
    std::string what;
  } res;
};

struct set_sdo_t
{
  struct Request
  {
    std::string desc;
    std::string module_id;
    uint16_t timeout_ms;

    uint16_t index;
    uint8_t subindex;

    enum sdotype_t : uint8_t
    {
      TYPE_U8 = 0,
      TYPE_U16 = 1,
      TYPE_U32 = 2,
      TYPE_U64 = 3,
      TYPE_I8 = 4,
      TYPE_I16 = 5,
      TYPE_I32 = 6,
      TYPE_I64 = 7
    } sdotype;

    std::array<uint8_t, 8> value;

  } req;

  struct Response
  {
    bool success;
    std::string what;
  } res;
};

template <typename R>
inline std::string to_string(const R& req)
{
  return std::string();
}

template <>
inline std::string to_string(const coe_master::get_sdo_t::Request& req)
{
  Json::Value json;
  json["dec"] = req.desc;
  json["module_id"] = req.module_id;
  json["timeout_ms"] = req.timeout_ms;
  json["index"] = req.index;
  json["subindex"] = req.subindex;
  json["sdotype"] = req.sdotype;
  json.setComment(std::string("// EOF"), Json::CommentPlacement::commentAfter);

  Json::StreamWriterBuilder stream_write_builder;
  return Json::writeString(stream_write_builder, json);
}

template <>
inline std::string to_string(const coe_master::get_sdo_t::Response& res)
{
  Json::Value json;
  json["success"] = res.success;
  json["value"] = Json::Value(Json::arrayValue);
  for (std::size_t i = 0; i != 8; i++)
  {
    Json::Value v = res.value[i];
    json["value"].append(v);
  }
  json["what"] = res.what;
  json.setComment(std::string("// EOF"), Json::CommentPlacement::commentAfter);

  Json::StreamWriterBuilder stream_write_builder;
  return Json::writeString(stream_write_builder, json);
}

template <>
inline std::string to_string(const coe_master::set_sdo_t::Request& req)
{
  Json::Value json;
  json["dec"] = req.desc;
  json["module_id"] = req.module_id;
  json["timeout_ms"] = req.timeout_ms;
  json["index"] = req.index;
  json["subindex"] = req.subindex;
  json["sdotype"] = Json::Value(Json::uintValue); 
   json["sdotype"] =req.sdotype;
  json["value"] = Json::Value(Json::arrayValue);
  for (std::size_t i = 0; i != 8; i++)
  {
    Json::Value v = req.value[i];
    json["value"].append(v);
  }
  json.setComment(std::string("// EOF"), Json::CommentPlacement::commentAfter);

  Json::StreamWriterBuilder stream_write_builder;
  auto str = Json::writeString(stream_write_builder, json);
  return str;
}

template <>
inline std::string to_string(const coe_master::set_sdo_t::Response& res)
{
  Json::Value json;
  json["success"] = res.success;
  json["what"] = res.what;
  json.setComment(std::string("// EOF"), Json::CommentPlacement::commentAfter);

  Json::StreamWriterBuilder stream_write_builder;
  return Json::writeString(stream_write_builder, json);
}

template <typename R>
inline bool validate(const std::string& income, std::string& what)
{
  Json::CharReaderBuilder char_reader_builder;
  Json::CharReader* reader = char_reader_builder.newCharReader();

  Json::Value json;
  std::string errors;

  bool parsingSuccessful = reader->parse(income.c_str(), income.c_str() + income.size(), &json, &errors);
  delete reader;

  if (!parsingSuccessful)
  {
    what = std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) +
           ": Failed to parse the JSON, errors: " + errors;
    return false;
  }

  bool ok = true;
  std::vector<std::string> mandatory_fields;
  std::string elab = "";
  if (typeid(R()) == typeid(coe_master::get_sdo_t::Request()))
  {
    elab = "coe_master::get_sdo_t::Request()";
    mandatory_fields = {"module_id", "timeout_ms", "index", "subindex", "sdotype"};
  }
  else if (typeid(R()) == typeid(coe_master::set_sdo_t::Request()))
  {
    elab = "coe_master::set_sdo_t::Request()";
    mandatory_fields = {"module_id", "timeout_ms", "index", "subindex", "sdotype", "value"};
  }
  if (typeid(R()) == typeid(coe_master::get_sdo_t::Response()))
  {
    elab = "coe_master::get_sdo_t::Response()";
    mandatory_fields = {"success", "value"};
  }
  else if (typeid(R()) == typeid(coe_master::set_sdo_t::Response()))
  {
    elab = "coe_master::set_sdo_t::Response()";
    mandatory_fields = {"success"};
  }

  what = "";
  std::stringstream ss;
  ss << "******************* VALIDATE:" << elab  << std::endl;
  ss << json << std::endl;
  ss << "mandatory fields: " ;
  for (auto const& f : mandatory_fields)
  {
    ss << f << ",";
    if (!json.isMember(f))
    {
      what += "'" + f + "' ";
      ok = false;
    }
  }
  std::cout << ss.str() << std::endl;
  if(!ok)
  {
    what = "In the JSON message are missing the fields: " + what;
  }

  return ok;
}

template <typename R>
inline Json::Value to_json(const std::string& income)
{
  std::string what;
  if(!validate<R>(income, what))
  {
    throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) +
                                         ": Failed to parse the JSON, errors: " + what)
                                 .c_str());
  }

  Json::CharReaderBuilder char_reader_builder;
  Json::CharReader* reader = char_reader_builder.newCharReader();

  Json::Value json;
  std::string errors;

  bool parsingSuccessful = reader->parse(income.c_str(), income.c_str() + income.size(), &json, &errors);
  delete reader;

  if (!parsingSuccessful)
  {
    throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) +
                                         ": Failed to parse the JSON, errors: " + errors)
                                 .c_str());
  }
  return json;
}

template <typename R>
inline R from_string(const std::string&)
{
  return R();
}
/**
 * @brief
 *
 * @param req
 * @return std::string
 */
template <>
inline coe_master::get_sdo_t::Request from_string(const std::string& str)
{
  coe_master::get_sdo_t::Request req;
  
  Json::Value json = to_json<coe_master::get_sdo_t::Request>(str);
  
  req.desc = json.get("desc", "").asString();
  req.module_id = json.get("module_id", "").asString();
  req.timeout_ms = json.get("timeout_ms", 0).asInt();
  req.index = json.get("index", 0).asInt();
  req.subindex = json.get("subindex", 0).asInt();
  req.sdotype = coe_master::get_sdo_t::Request::sdotype_t(json.get("sdotype", 0).asUInt());
  return req;
}

template <>
inline coe_master::get_sdo_t::Response from_string(const std::string& str)
{
  coe_master::get_sdo_t::Response res;
  Json::Value json = to_json<coe_master::get_sdo_t::Response>(str);

  res.success = json["success"].asBool();
  res.what = json["what"].asString();
  Json::Value tmp = json["value"];
  for (int i = 0; i < 8; i++) res.value[i] = tmp[i].asUInt();

  return res;
}

template <>
inline coe_master::set_sdo_t::Request from_string(const std::string& str)
{
  coe_master::set_sdo_t::Request req;
  Json::Value json = to_json<coe_master::set_sdo_t::Request>(str);

  req.desc = json.get("desc", "").asString();
  req.module_id = json.get("module_id", "").asString();
  req.timeout_ms = json.get("timeout_ms", 0).asUInt();
  req.index = json.get("index", 0).asUInt();
  req.subindex = json.get("subindex", 0).asUInt();
  req.sdotype = coe_master::set_sdo_t::Request::sdotype_t(json.get("sdotype", 0).asUInt());

  for (int i = 0; i < 8; i++) req.value[i] = json["value"][i].asUInt();

  return req;
}

template <>
inline coe_master::set_sdo_t::Response from_string(const std::string& str)
{
  coe_master::set_sdo_t::Response res;
  Json::Value json = to_json<coe_master::set_sdo_t::Response>(str);

  res.success = json["success"].asBool();
  res.what = json["what"].asString();

  return res;
}

}  // namespace coe_master

#endif  // COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC_MSGS
