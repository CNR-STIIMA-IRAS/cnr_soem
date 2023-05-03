/**
 *
 * @file coe_dictionary.h
 * @brief Namespace storing the structures to describe the dictionary according
 * to CiA CANopen specifications
 *
 */

#ifndef COE_CORE_INCLUDE_COE_CORE_COE_BASE
#define COE_CORE_INCLUDE_COE_CORE_COE_BASE

#include <algorithm>
#include <assert.h>
#include <boost/algorithm/string.hpp>
#include <cstring>
#include <ethercattype.h>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <typeindex>
#include <typeinfo>

#include <boost/array.hpp>
#include <coe_core/coe_cob_types.h>
#include <coe_core/coe_string_utilities.h>
#include <string>

namespace coe_core
{

  enum access_t
  {
    WRITE,
    READ
  };
  enum on_failure_t
  {
    FATAL,
    WARNINIG
  };

  /**
   * @class BaseDataObjectEntry Abstract Class
   */
class BaseDataObjectEntry
{
private:
  const uint16_t index_;   ///< address of the object
  const uint8_t subindex_; ///< sub-index of the object
  std::string id_;         ///< string that identify what stores this object
  EcTypes::map::const_iterator type_info_;

public:
  typedef std::shared_ptr<BaseDataObjectEntry> Ptr;

  BaseDataObjectEntry(const uint16_t index, const int subindex, const std::string id, ec_datatype data_type)
      : index_(index), subindex_(subindex), id_(id)
  {
    type_info_ = EcTypes::Map().find(data_type);
    if (type_info_ != EcTypes::Map().end())
    {
      throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
                                            ": Input Type not in the mapped ones")
                                    .c_str());
    }
  };

  // Access Methods
  uint16_t index() const { return index_; }
  uint8_t subindex() const { return subindex_; }
  uint32_t address() const
  {
    uint32_t ret = 0;
    ret = ((uint32_t)index_ << 16) | (uint32_t)subindex_;
    return ret;
  }
  std::string name() const { return id_; }
  ec_datatype type() const { return type_info_->first; }
  size_t sizeBits() const { return std::get<1>(type_info_->second); }
  size_t sizeBytes() const { return std::get<2>(type_info_->second); }

  // Pure virtual Methods
  virtual const uint8_t *data() const = 0;
  virtual uint8_t *data() = 0;

  virtual void operator>>(uint8_t *value) const = 0;
  virtual void operator<<(const uint8_t *value) = 0;

  // Utils
  template <typename T> T get() const
  {
    if (coe_core::getType<T>() != type())
    {
      throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
                                            ": Type Mismatch. Object type '" + coe_core::to_string(type(), false) +
                                            "', Return Type: '" + coe_core::to_string(getType<T>(), false) + "'")
                                    .c_str());
    }
    T ret;
    std::memcpy(&ret, data(), sizeBytes());
    return ret;
  }
  template <typename T> void set(const T &value)
  {
    if (coe_core::getType<T>() != type())
    {
      throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
                                                ": Type Mismatch. Object type '" + coe_core::to_string(type(), false) + 
                                                "', Return Type: '" + coe_core::to_string(getType<T>(), false) + "'")
                                    .c_str());
      assert(0);
    }
    std::memcpy(data(), &value, sizeBytes());
  }
  virtual bool operator==(const BaseDataObjectEntry &rhs) const
  {
    return (index_ == rhs.index()) && (subindex_ == rhs.subindex()) && (id_ == rhs.name()) &&
            (type_info_->first == rhs.type());
  }
  virtual std::string to_string(bool verbose = true) const
  {
    std::string ret = coe_core::to_string_hex(index_) + ":" + std::to_string(subindex_);
    if (verbose)
      ret += "\t[" + coe_core::dtype2string(type(), true) + "/" +
              (std::get<1>(type_info_->second) < 10 ? "0" + std::to_string(std::get<1>(type_info_->second))
                                                    : std::to_string(std::get<1>(type_info_->second))) +
              "B]";
    return ret;
  }
};

typedef BaseDataObjectEntry::Ptr BaseDataObjectEntryPtr;

/**
 * @class DataObjectEntry
 */
template <typename T> class DataObjectEntry : public BaseDataObjectEntry
{
private:
  T val_;

public:
  DataObjectEntry(const uint16_t index, const int subindex, const std::string id, const T val = T())
      : BaseDataObjectEntry(index, subindex, id, getType<T>()), val_(val)
  {
  }

  const uint8_t *data() const { return (const uint8_t *)(&val_); }
  uint8_t *data() { return (uint8_t *)(&val_); }

  void operator>>(uint8_t *value) const { std::memcpy(&value[0], &val_, sizeBytes()); }
  void operator<<(const uint8_t *value) { std::memcpy(&val_, &value[0], sizeBytes()); }

  const T &value() const { return val_; }
  T &value() { return val_; }

  std::string to_string(bool verbose = true) const
  {
    return verbose
                ? BaseDataObjectEntry::to_string(verbose) + "\tval: " + coe_core::to_string_hex(val_) + "\t" + name()
                : coe_core::to_string_hex(val_);
  }
};

  template <typename T> DataObjectEntry<T> *cast(BaseDataObjectEntry *in)
  {
    if (getType<T>() != in->type())
    {
      throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
                                                ": Type Mismatch. Object type '" + coe_core::to_string(in->type(), false) + 
                                                "', Return Type: '" + coe_core::to_string(getType<T>(), false) + "'")
                                    .c_str());
    }
    return static_cast<DataObjectEntry<T> *>(in);
  }

  template <typename T> std::shared_ptr<DataObjectEntry<T>> cast(BaseDataObjectEntryPtr in)
  {
    assert(getType<T>() == in->type());
    std::shared_ptr<DataObjectEntry<T>> ret(static_cast<DataObjectEntry<T> *>(in.get()));
    return ret;
  }

/**
 * @class WeakDataObject
 */
class WeakDataObject
{
public:
    typedef std::shared_ptr<WeakDataObject> Ptr;
    typedef std::vector<coe_core::BaseDataObjectEntryPtr> BDOList;
    typedef BDOList::iterator iterator;
    typedef BDOList::const_iterator const_iterator;

    WeakDataObject() : finalized_(false) {}
    void clear()
    {
      cob_list_.clear();
      finalized_ = false;
    }
    iterator begin()
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.begin();
    }
    iterator end()
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.end();
    }

    const_iterator begin() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.begin();
    }
    const_iterator end() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.end();
    }

    const_iterator cbegin() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.cbegin();
    }
    const_iterator cend() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.cend();
    }

    const coe_core::BaseDataObjectEntryPtr front() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.front();
    }
    const coe_core::BaseDataObjectEntryPtr back() const
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.back();
    }

    coe_core::BaseDataObjectEntryPtr front()
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.front();
    }
    coe_core::BaseDataObjectEntryPtr back()
    {
      if (!finalized_)
        throw std::runtime_error("The WeakDataObject has not been finalized. Abort.");
      return cob_list_.back();
    }

    const coe_core::BaseDataObjectEntryPtr &at(size_t i) const;
    coe_core::BaseDataObjectEntryPtr &at(size_t i);

    const_iterator find(uint16_t index_pdo_entry, int8_t subindex_pdo_entry = -1) const;
    iterator find(uint16_t index_pdo_entry, int8_t subindex_pdo_entry = -1);

    size_t nEntries() const;

    virtual bool push_back(coe_core::BaseDataObjectEntryPtr cob, std::string& what);
    void finalize();
    bool isFinalized() const;

    bool operator==(const WeakDataObject &rhs) const;
    virtual std::string to_string(bool verbose) const;

protected:
    BDOList cob_list_;
    bool finalized_;
  };

  typedef WeakDataObject::Ptr WeakDataObjectPtr;

  class DataObject : public WeakDataObject
  {
public:
    typedef std::shared_ptr<DataObject> Ptr;

    virtual bool push_back(coe_core::BaseDataObjectEntryPtr cob, std::string& what)
    {
      if ((cob_list_.size() == 0) || (cob_list_.front()->index() == cob->index()))
      {
        return WeakDataObject::push_back(cob, what);
      }
      
      what = "Index not in the list";
      return false;
    }

    size_t index() const
    {
      if ((!finalized_) || (cob_list_.size() == 0))
        throw std::runtime_error("WeakDataObject::index() const | The CobVector "
                                 "has not been finalized. Abort.");

      return cob_list_.front()->index();
    }
  };

  typedef DataObject::Ptr DataObjectPtr;

} // namespace coe_core

#endif /* COE_CORE_INCLUDE_COE_CORE_COE_BASE */
