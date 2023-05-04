#include <chrono>
#include <numeric>
#include <coe_core/coe_pdo.h>

namespace coe_core
{

void Pdo::flush(uint8_t *data, bool prepend_time) const
{
  if (size_bits_map_.size() != cob_list_.size())
    throw std::runtime_error("The bitmap for the packet struct has not yet defined. ");

  uint8_t *v = &data[0];
  if (prepend_time)
  {
    std::memcpy(v, (uint8_t *)&time_, sizeof(time_));
    v += sizeof(time_);
  }

  for (auto const &cob : cob_list_)
  {

    if (cob->sizeBits() % 8 == 0)
    {
      std::memcpy(&v[start_bytes_map_.at(cob->address())], cob->data(), cob->sizeBytes());
    }
    else if (cob->sizeBits() != 1)
    {
      throw std::runtime_error("Uff, ancora da fare, anche se è molto strano");
    }
    else
    {
      if ((cob->type() != ECT_BOOLEAN) && (cob->type() != ECT_BIT1) && (cob->type() != ECT_BIT2) &&
          (cob->type() != ECT_BIT3) && (cob->type() != ECT_BIT4) && (cob->type() != ECT_BIT5) &&
          (cob->type() != ECT_BIT6) && (cob->type() != ECT_BIT7) && (cob->type() != ECT_BIT8))
      {
        throw std::runtime_error("Merda, ancora da fare, anche se è molto strano");
      }

      uint8_t mask = 1 << start_bits_map_.at(cob->address());
      uint8_t ret = (v[start_bytes_map_.at(cob->address())]) & (~mask);
      uint8_t val;

      (*cob) >> &val;
      val = val << start_bits_map_.at(cob->address());

      ret = ret | val;

      std::memcpy(&v[start_bytes_map_.at(cob->address())], &ret, 1);
    }
  }
}

void Pdo::update(const uint8_t *data, bool prepended_time)
{
  if (!finalized_)
    throw std::runtime_error("The PdoVector has not been finalized. Abort.");

  uint8_t raw_buffer[1024];
  std::memset(raw_buffer, 0x0, 1024 * sizeof(uint8_t));

  uint8_t *ptr = &raw_buffer[0];
  std::memcpy(ptr, data, prepended_time ? (sizeof(double) + dim_bytes_) : (dim_bytes_));

  if (prepended_time)
  {
    time_ = *(double *)&ptr[0];
    ptr += sizeof(double);
  }
  else
  {
    time_ = double(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count()) /
            1.0e6;
  }

  if (size_bits_map_.size() != cob_list_.size())
    throw std::runtime_error("The bitmap for the packet struct has not yet defined. ");

  for (auto &cob : cob_list_)
  {
    if (cob->sizeBits() % 8 == 0)
    {
      std::memcpy(cob->data(), &ptr[start_bytes_map_.at(cob->address())], cob->sizeBits() / 8);
    }
    else if (cob->sizeBits() != 1)
    {
      throw std::runtime_error("Uff, ancora da fare, anche se è molto strano");
    }
    else
    {
      if ((cob->type() != ECT_BOOLEAN) && (cob->type() != ECT_BIT1) && (cob->type() != ECT_BIT2) &&
          (cob->type() != ECT_BIT3) && (cob->type() != ECT_BIT4) && (cob->type() != ECT_BIT5) &&
          (cob->type() != ECT_BIT6) && (cob->type() != ECT_BIT7) && (cob->type() != ECT_BIT8))
      {
        throw std::runtime_error("Merda, ancora da fare, anche se è molto strano");
      }

      uint8_t mask = 1 << start_bits_map_.at(cob->address());
      uint8_t val = ptr[start_bytes_map_.at(cob->address())];
      val = (val & mask);
      val = val > 0 ? 1 : 0;

      (*cob) << &val;
    }
  }
}

std::string Pdo::to_string(bool verbose) const
{
  std::string ret = (isRxPdo() ? "[ RxPDO ] " : "[ TxPDO ] ") + std::to_string(nEntries()) + "# entries\n";
  ret += "Number of Entries: " + std::to_string(cob_list_.size()) + "\n";

  for (auto const &f : cob_list_)
  {
    std::string offset_bytes = (start_bytes_map_.find(f->address()) == start_bytes_map_.end())
                                    ? "N/A"
                                    : std::to_string(start_bytes_map_.at(f->address()));
    std::string offset_bits = (start_bits_map_.find(f->address()) == start_bits_map_.end())
                                  ? "N/A"
                                  : std::to_string(start_bits_map_.at(f->address()));

    ret += "offset " + offset_bytes + ":" + offset_bits + " -  index:" + f->to_string() + "\n";
  }
  ret += "Packed size   [B]: " + std::to_string(nBytes(true));
  return ret;
}

std::size_t Pdo::nBytes(bool packed) const
{
  std::size_t ret = 0;

  if (!finalized_)
    ret = 9999999;
  else if (cob_list_.size() == 0)
    ret = 0;
  else if (!packed)
    ret = std::accumulate(cob_list_.begin(), cob_list_.end(), 0,
                          [](std::size_t s, coe_core::BaseDataObjectEntryPtr cob) { return s + cob->sizeBytes(); });
  else
    ret = dim_bytes_;

  return ret;
}

}  // namespace coe_core
