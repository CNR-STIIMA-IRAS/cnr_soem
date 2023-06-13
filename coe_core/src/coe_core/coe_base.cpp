#include <assert.h>
#include <coe_core/coe_base.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_core/ds402/coe_xfsm_utilities.h>
#include <ethercattype.h>
#include <algorithm>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <typeindex>
#include <typeinfo>

namespace coe_core
{

bool WeakDataObject::push_back(coe_core::BaseDataObjectEntryPtr cob, std::string& what)
{
  finalized_ = false;

  if (cob_list_.end() != std::find_if(cob_list_.begin(),
                                      cob_list_.end(),
                                      [&cob](coe_core::BaseDataObjectEntryPtr& c)
                                      { return (c->subindex() == cob->subindex()) && (c->index() == cob->index()); }))
  {
    what = "COB " + coe_core::to_string_hex(cob->address()) + " already assigned. Abort.";
    return false;
  }

  cob_list_.push_back(cob);
  return true;
}

const coe_core::BaseDataObjectEntryPtr& WeakDataObject::at(std::size_t i) const
{
  if (!finalized_)
    throw std::runtime_error("WeakDataObject::at() const | The CobVector has not been finalized. Abort.");
  return cob_list_.at(i);
}

coe_core::BaseDataObjectEntryPtr& WeakDataObject::at(std::size_t i)
{
  if (!finalized_) throw std::runtime_error("WeakDataObject::at() | The CobVector has not been finalized. Abort.");
  return cob_list_.at(i);
}

void WeakDataObject::finalize() { finalized_ = true; }

std::size_t WeakDataObject::nEntries() const { return cob_list_.size(); }

bool WeakDataObject::isFinalized() const { return finalized_; }

bool WeakDataObject::operator==(const WeakDataObject& rhs) const
{
  for (auto const& obj : rhs)
  {
    auto it = std::find_if(
        cob_list_.begin(), cob_list_.end(), [&](const BaseDataObjectEntryPtr& cob) { return *cob == *obj; });
    if (it == cob_list_.end())
    {
      return false;
    }
  }
  return true;
}

std::string WeakDataObject::to_string() const
{
  std::string ret;
  for (auto const& f : cob_list_)
  {
    ret += f->to_string() + "\n";
  }
  return ret;
}

WeakDataObject::iterator WeakDataObject::find(uint16_t index_pdo_entry, int8_t subindex_pdo_entry)
{
  auto it = std::find_if(cob_list_.begin(),
                         cob_list_.end(),
                         [&](coe_core::BaseDataObjectEntryPtr p)
                         { return index_pdo_entry == p->index() && subindex_pdo_entry == p->subindex(); });
  auto jt = std::find_if(cob_list_.begin(),
                         cob_list_.end(),
                         [&](coe_core::BaseDataObjectEntryPtr p) { return index_pdo_entry == p->index(); });

  return (subindex_pdo_entry == -1) ? jt : it;
}

WeakDataObject::const_iterator WeakDataObject::find(uint16_t index_pdo_entry, int8_t subindex_pdo_entry) const
{
  auto const it = std::find_if(cob_list_.begin(),
                               cob_list_.end(),
                               [&](const coe_core::BaseDataObjectEntryPtr& p)
                               { return index_pdo_entry == p->index() && subindex_pdo_entry == p->subindex(); });
  auto const jt =
      std::find_if(cob_list_.begin(),
                   cob_list_.end(),
                   [&](const coe_core::BaseDataObjectEntryPtr& p) { return index_pdo_entry == p->index(); });
  return (subindex_pdo_entry == -1) ? jt : it;
}

}  // namespace coe_core
