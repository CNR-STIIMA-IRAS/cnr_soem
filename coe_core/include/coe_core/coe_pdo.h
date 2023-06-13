#ifndef SRC_CNR_SOEM_COE_CORE_INCLUDE_COE_CORE_COE_PDO
#define SRC_CNR_SOEM_COE_CORE_INCLUDE_COE_CORE_COE_PDO

#include <type_traits>
#include <ethercattype.h>
#include <algorithm>
#include <exception>
#include <map>
#include <string>
#include <typeinfo>
#include <vector>

#include <coe_core/coe_base.h>

namespace coe_core
{

enum class PdoType : std::size_t
{
  RX_PDO = 0, TX_PDO = 1
};

template<std::size_t N> 
struct toPdoType
{
  static const typename std::enable_if<N==ECT_SDO_RXPDOASSIGN || N==ECT_SDO_TXPDOASSIGN, PdoType>::type 
    value;
};

template<> 
struct toPdoType<ECT_SDO_RXPDOASSIGN>
{
  static const PdoType  value = PdoType::RX_PDO;
};


template<> 
struct toPdoType<ECT_SDO_TXPDOASSIGN>
{
  static const PdoType value = PdoType::TX_PDO;
};



template<PdoType PDO_TYPE> 
struct toUint
{
  static const std::size_t value = 0;
};

template<> 
struct toUint<PdoType::RX_PDO>
{
  static const std::size_t value = ECT_SDO_RXPDOASSIGN;
};


template<> 
struct toUint<PdoType::TX_PDO>
{
  static const std::size_t value = ECT_SDO_TXPDOASSIGN;
};

template<PdoType PDO_TYPE>
class Pdo : public WeakDataObject
{
public:
  typedef std::shared_ptr<Pdo> Ptr;
  typedef std::shared_ptr<const Pdo> ConstPtr;

  Pdo() = default;
  Pdo(const Pdo&) = delete;
  Pdo(Pdo&&) = delete;

  const coe_core::BaseDataObjectEntryPtr &subindex(std::size_t i) const { return at(i); }
  coe_core::BaseDataObjectEntryPtr &subindex(std::size_t i) { return at(i); };

  void update(const uint8_t *data, bool prepended_time);
  void flush(uint8_t *data, bool prepended_time) const;

  double time() const { return time_; }
  std::size_t nBytes(bool packed) const;

  //bool operator==(const Pdo &rhs) const;
  std::string to_string() const;

  std::map<std::size_t, std::size_t> start_bits_map_;
  std::map<std::size_t, std::size_t> start_bytes_map_;
  std::map<std::size_t, std::size_t> size_bits_map_;

  void setPackedBytesLenght(const std::size_t dim_bytes) { dim_bytes_ = dim_bytes; }

private:
  const uint16_t addr_;
  double time_;
  std::size_t dim_bytes_;
};


template<PdoType PDO_TYPE>
using PdoPtr = typename Pdo<PDO_TYPE>::Ptr;

template<PdoType PDO_TYPE>
using PdoConstPtr = typename Pdo<PDO_TYPE>::ConstPtr;

using RxPdo = Pdo<PdoType::RX_PDO>;
using TxPdo = Pdo<PdoType::TX_PDO>;

using RxPdoPtr = PdoPtr<PdoType::RX_PDO>;
using TxPdoPtr = PdoPtr<PdoType::TX_PDO>;

using RxPdoConstPtr = PdoConstPtr<PdoType::RX_PDO>;
using TxPdoConstPtr = PdoConstPtr<PdoType::TX_PDO>;


template<PdoType PDO_TYPE>
inline std::vector<DataObjectPtr> split(const Pdo<PDO_TYPE> &pdo, std::string &what)
{
  std::vector<DataObjectPtr> ret;
  for (BaseDataObjectEntryPtr cob : pdo)
  {
    bool new_data_object = (ret.size() == 0) || (!ret.back()->push_back(cob, what));
    if (new_data_object)
    {
      DataObjectPtr cob_list(new DataObject());
      if (cob_list->push_back(cob, what))
      {
        ret.push_back(cob_list);
      }
    }
  }
  for (auto &dop : ret) dop->finalize();
  return ret;
}



///< @endcond NOSKIINLINE

}  // namespace coe_core

#include <coe_core/impl/coe_pdo_impl.hpp>

#endif  /* SRC_CNR_SOEM_COE_CORE_INCLUDE_COE_CORE_COE_PDO */
