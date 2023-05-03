#ifndef COE_CORE_COE_PDO__H
#define COE_CORE_COE_PDO__H

#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <ethercattype.h>
#include <exception>
#include <typeinfo>

#include <coe_core/coe_base.h>

namespace coe_core
{

/**
 *
 * @class PdoVector
 *
 *
 *
 */
class Pdo : public WeakDataObject
{
public:
  typedef std::shared_ptr<Pdo> Ptr;
  typedef std::shared_ptr<const Pdo> ConstPtr;

  Pdo(const uint16_t addr) : addr_(addr)
  {
    // nothing do do so far
  }

  bool isRxPdo() const { return addr_ == ECT_SDO_RXPDOASSIGN; }
  bool isTxPdo() const { return addr_ == ECT_SDO_TXPDOASSIGN; }

  const coe_core::BaseDataObjectEntryPtr &subindex(size_t i) const { return at(i); }
  coe_core::BaseDataObjectEntryPtr &subindex(size_t i) { return at(i); };

  void update(const uint8_t *data, bool prepended_time);
  void flush(uint8_t *data, bool prepended_time) const;

  double time() const { return time_; }
  size_t nBytes(bool packed) const;

  bool operator==(const Pdo &rhs) const;
  std::string to_string(bool verbose = true) const;

  std::map<size_t, size_t> start_bits_map_;
  std::map<size_t, size_t> start_bytes_map_;
  std::map<size_t, size_t> size_bits_map_;

  void setPackedBytesLenght(const size_t dim_bytes) { dim_bytes_ = dim_bytes; }

private:
  const uint16_t addr_;
  double time_;
  size_t dim_bytes_;
};

typedef Pdo::Ptr PdoPtr;
typedef Pdo::ConstPtr ConstPdoPtr;

inline std::vector<DataObjectPtr> split(const Pdo &pdo, std::string& what)
{
  std::vector<DataObjectPtr> ret;
  for (BaseDataObjectEntryPtr cob : pdo)
  {
    bool new_data_object = (ret.size() == 0) || (!ret.back()->push_back(cob,what));
    if (new_data_object)
    {
      DataObjectPtr cob_list(new DataObject());
      if(cob_list->push_back(cob,what))
      {
        ret.push_back(cob_list);
      }
    }
  }
  for (auto &dop : ret)
    dop->finalize();
  return ret;
}

///< @endcond NOSKIINLINE

} // namespace coe_core

#endif  // COE_CORE_COE_PDO__H
