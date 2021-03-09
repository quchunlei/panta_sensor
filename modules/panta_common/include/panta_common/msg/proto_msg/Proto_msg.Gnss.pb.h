// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Proto_msg.Gnss.proto

#ifndef PROTOBUF_Proto_5fmsg_2eGnss_2eproto__INCLUDED
#define PROTOBUF_Proto_5fmsg_2eGnss_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace Proto_msg {
class Gnss;
class GnssDefaultTypeInternal;
extern GnssDefaultTypeInternal _Gnss_default_instance_;
}  // namespace Proto_msg

namespace Proto_msg {

namespace protobuf_Proto_5fmsg_2eGnss_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_Proto_5fmsg_2eGnss_2eproto

// ===================================================================

class Gnss : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:Proto_msg.Gnss) */ {
 public:
  Gnss();
  virtual ~Gnss();

  Gnss(const Gnss& from);

  inline Gnss& operator=(const Gnss& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Gnss(Gnss&& from) noexcept
    : Gnss() {
    *this = ::std::move(from);
  }

  inline Gnss& operator=(Gnss&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Gnss& default_instance();

  static inline const Gnss* internal_default_instance() {
    return reinterpret_cast<const Gnss*>(
               &_Gnss_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Gnss* other);
  friend void swap(Gnss& a, Gnss& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Gnss* New() const PROTOBUF_FINAL { return New(NULL); }

  Gnss* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Gnss& from);
  void MergeFrom(const Gnss& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Gnss* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated double pos = 8;
  int pos_size() const;
  void clear_pos();
  static const int kPosFieldNumber = 8;
  double pos(int index) const;
  void set_pos(int index, double value);
  void add_pos(double value);
  const ::google::protobuf::RepeatedField< double >&
      pos() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_pos();

  // repeated double pos_cov = 9;
  int pos_cov_size() const;
  void clear_pos_cov();
  static const int kPosCovFieldNumber = 9;
  double pos_cov(int index) const;
  void set_pos_cov(int index, double value);
  void add_pos_cov(double value);
  const ::google::protobuf::RepeatedField< double >&
      pos_cov() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_pos_cov();

  // repeated double orien = 10;
  int orien_size() const;
  void clear_orien();
  static const int kOrienFieldNumber = 10;
  double orien(int index) const;
  void set_orien(int index, double value);
  void add_orien(double value);
  const ::google::protobuf::RepeatedField< double >&
      orien() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_orien();

  // repeated double oeirn_cov = 11;
  int oeirn_cov_size() const;
  void clear_oeirn_cov();
  static const int kOeirnCovFieldNumber = 11;
  double oeirn_cov(int index) const;
  void set_oeirn_cov(int index, double value);
  void add_oeirn_cov(double value);
  const ::google::protobuf::RepeatedField< double >&
      oeirn_cov() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_oeirn_cov();

  // repeated double linear_vel = 12;
  int linear_vel_size() const;
  void clear_linear_vel();
  static const int kLinearVelFieldNumber = 12;
  double linear_vel(int index) const;
  void set_linear_vel(int index, double value);
  void add_linear_vel(double value);
  const ::google::protobuf::RepeatedField< double >&
      linear_vel() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_linear_vel();

  // repeated double linear_vel_cov = 13;
  int linear_vel_cov_size() const;
  void clear_linear_vel_cov();
  static const int kLinearVelCovFieldNumber = 13;
  double linear_vel_cov(int index) const;
  void set_linear_vel_cov(int index, double value);
  void add_linear_vel_cov(double value);
  const ::google::protobuf::RepeatedField< double >&
      linear_vel_cov() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_linear_vel_cov();

  // optional string parent_frame_id = 3;
  bool has_parent_frame_id() const;
  void clear_parent_frame_id();
  static const int kParentFrameIdFieldNumber = 3;
  const ::std::string& parent_frame_id() const;
  void set_parent_frame_id(const ::std::string& value);
  #if LANG_CXX11
  void set_parent_frame_id(::std::string&& value);
  #endif
  void set_parent_frame_id(const char* value);
  void set_parent_frame_id(const char* value, size_t size);
  ::std::string* mutable_parent_frame_id();
  ::std::string* release_parent_frame_id();
  void set_allocated_parent_frame_id(::std::string* parent_frame_id);

  // optional string frame_id = 4;
  bool has_frame_id() const;
  void clear_frame_id();
  static const int kFrameIdFieldNumber = 4;
  const ::std::string& frame_id() const;
  void set_frame_id(const ::std::string& value);
  #if LANG_CXX11
  void set_frame_id(::std::string&& value);
  #endif
  void set_frame_id(const char* value);
  void set_frame_id(const char* value, size_t size);
  ::std::string* mutable_frame_id();
  ::std::string* release_frame_id();
  void set_allocated_frame_id(::std::string* frame_id);

  // optional double timestamp = 1;
  bool has_timestamp() const;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 1;
  double timestamp() const;
  void set_timestamp(double value);

  // optional uint32 seq = 2;
  bool has_seq() const;
  void clear_seq();
  static const int kSeqFieldNumber = 2;
  ::google::protobuf::uint32 seq() const;
  void set_seq(::google::protobuf::uint32 value);

  // optional uint32 status = 5;
  bool has_status() const;
  void clear_status();
  static const int kStatusFieldNumber = 5;
  ::google::protobuf::uint32 status() const;
  void set_status(::google::protobuf::uint32 value);

  // optional uint32 type = 6;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 6;
  ::google::protobuf::uint32 type() const;
  void set_type(::google::protobuf::uint32 value);

  // optional uint32 sat_cnt = 7;
  bool has_sat_cnt() const;
  void clear_sat_cnt();
  static const int kSatCntFieldNumber = 7;
  ::google::protobuf::uint32 sat_cnt() const;
  void set_sat_cnt(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:Proto_msg.Gnss)
 private:
  void set_has_timestamp();
  void clear_has_timestamp();
  void set_has_seq();
  void clear_has_seq();
  void set_has_parent_frame_id();
  void clear_has_parent_frame_id();
  void set_has_frame_id();
  void clear_has_frame_id();
  void set_has_status();
  void clear_has_status();
  void set_has_type();
  void clear_has_type();
  void set_has_sat_cnt();
  void clear_has_sat_cnt();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< double > pos_;
  ::google::protobuf::RepeatedField< double > pos_cov_;
  ::google::protobuf::RepeatedField< double > orien_;
  ::google::protobuf::RepeatedField< double > oeirn_cov_;
  ::google::protobuf::RepeatedField< double > linear_vel_;
  ::google::protobuf::RepeatedField< double > linear_vel_cov_;
  ::google::protobuf::internal::ArenaStringPtr parent_frame_id_;
  ::google::protobuf::internal::ArenaStringPtr frame_id_;
  double timestamp_;
  ::google::protobuf::uint32 seq_;
  ::google::protobuf::uint32 status_;
  ::google::protobuf::uint32 type_;
  ::google::protobuf::uint32 sat_cnt_;
  friend struct protobuf_Proto_5fmsg_2eGnss_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Gnss

// optional double timestamp = 1;
inline bool Gnss::has_timestamp() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Gnss::set_has_timestamp() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Gnss::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Gnss::clear_timestamp() {
  timestamp_ = 0;
  clear_has_timestamp();
}
inline double Gnss::timestamp() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.timestamp)
  return timestamp_;
}
inline void Gnss::set_timestamp(double value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.timestamp)
}

// optional uint32 seq = 2;
inline bool Gnss::has_seq() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Gnss::set_has_seq() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Gnss::clear_has_seq() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Gnss::clear_seq() {
  seq_ = 0u;
  clear_has_seq();
}
inline ::google::protobuf::uint32 Gnss::seq() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.seq)
  return seq_;
}
inline void Gnss::set_seq(::google::protobuf::uint32 value) {
  set_has_seq();
  seq_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.seq)
}

// optional string parent_frame_id = 3;
inline bool Gnss::has_parent_frame_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Gnss::set_has_parent_frame_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Gnss::clear_has_parent_frame_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Gnss::clear_parent_frame_id() {
  parent_frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_parent_frame_id();
}
inline const ::std::string& Gnss::parent_frame_id() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.parent_frame_id)
  return parent_frame_id_.GetNoArena();
}
inline void Gnss::set_parent_frame_id(const ::std::string& value) {
  set_has_parent_frame_id();
  parent_frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.parent_frame_id)
}
#if LANG_CXX11
inline void Gnss::set_parent_frame_id(::std::string&& value) {
  set_has_parent_frame_id();
  parent_frame_id_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:Proto_msg.Gnss.parent_frame_id)
}
#endif
inline void Gnss::set_parent_frame_id(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_parent_frame_id();
  parent_frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:Proto_msg.Gnss.parent_frame_id)
}
inline void Gnss::set_parent_frame_id(const char* value, size_t size) {
  set_has_parent_frame_id();
  parent_frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.Gnss.parent_frame_id)
}
inline ::std::string* Gnss::mutable_parent_frame_id() {
  set_has_parent_frame_id();
  // @@protoc_insertion_point(field_mutable:Proto_msg.Gnss.parent_frame_id)
  return parent_frame_id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Gnss::release_parent_frame_id() {
  // @@protoc_insertion_point(field_release:Proto_msg.Gnss.parent_frame_id)
  clear_has_parent_frame_id();
  return parent_frame_id_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Gnss::set_allocated_parent_frame_id(::std::string* parent_frame_id) {
  if (parent_frame_id != NULL) {
    set_has_parent_frame_id();
  } else {
    clear_has_parent_frame_id();
  }
  parent_frame_id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), parent_frame_id);
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.Gnss.parent_frame_id)
}

// optional string frame_id = 4;
inline bool Gnss::has_frame_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Gnss::set_has_frame_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Gnss::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Gnss::clear_frame_id() {
  frame_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_frame_id();
}
inline const ::std::string& Gnss::frame_id() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.frame_id)
  return frame_id_.GetNoArena();
}
inline void Gnss::set_frame_id(const ::std::string& value) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.frame_id)
}
#if LANG_CXX11
inline void Gnss::set_frame_id(::std::string&& value) {
  set_has_frame_id();
  frame_id_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:Proto_msg.Gnss.frame_id)
}
#endif
inline void Gnss::set_frame_id(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:Proto_msg.Gnss.frame_id)
}
inline void Gnss::set_frame_id(const char* value, size_t size) {
  set_has_frame_id();
  frame_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.Gnss.frame_id)
}
inline ::std::string* Gnss::mutable_frame_id() {
  set_has_frame_id();
  // @@protoc_insertion_point(field_mutable:Proto_msg.Gnss.frame_id)
  return frame_id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Gnss::release_frame_id() {
  // @@protoc_insertion_point(field_release:Proto_msg.Gnss.frame_id)
  clear_has_frame_id();
  return frame_id_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Gnss::set_allocated_frame_id(::std::string* frame_id) {
  if (frame_id != NULL) {
    set_has_frame_id();
  } else {
    clear_has_frame_id();
  }
  frame_id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), frame_id);
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.Gnss.frame_id)
}

// optional uint32 status = 5;
inline bool Gnss::has_status() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Gnss::set_has_status() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Gnss::clear_has_status() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Gnss::clear_status() {
  status_ = 0u;
  clear_has_status();
}
inline ::google::protobuf::uint32 Gnss::status() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.status)
  return status_;
}
inline void Gnss::set_status(::google::protobuf::uint32 value) {
  set_has_status();
  status_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.status)
}

// optional uint32 type = 6;
inline bool Gnss::has_type() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Gnss::set_has_type() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Gnss::clear_has_type() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Gnss::clear_type() {
  type_ = 0u;
  clear_has_type();
}
inline ::google::protobuf::uint32 Gnss::type() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.type)
  return type_;
}
inline void Gnss::set_type(::google::protobuf::uint32 value) {
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.type)
}

// optional uint32 sat_cnt = 7;
inline bool Gnss::has_sat_cnt() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Gnss::set_has_sat_cnt() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Gnss::clear_has_sat_cnt() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Gnss::clear_sat_cnt() {
  sat_cnt_ = 0u;
  clear_has_sat_cnt();
}
inline ::google::protobuf::uint32 Gnss::sat_cnt() const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.sat_cnt)
  return sat_cnt_;
}
inline void Gnss::set_sat_cnt(::google::protobuf::uint32 value) {
  set_has_sat_cnt();
  sat_cnt_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.sat_cnt)
}

// repeated double pos = 8;
inline int Gnss::pos_size() const {
  return pos_.size();
}
inline void Gnss::clear_pos() {
  pos_.Clear();
}
inline double Gnss::pos(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.pos)
  return pos_.Get(index);
}
inline void Gnss::set_pos(int index, double value) {
  pos_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.pos)
}
inline void Gnss::add_pos(double value) {
  pos_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.pos)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::pos() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.pos)
  return pos_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_pos() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.pos)
  return &pos_;
}

// repeated double pos_cov = 9;
inline int Gnss::pos_cov_size() const {
  return pos_cov_.size();
}
inline void Gnss::clear_pos_cov() {
  pos_cov_.Clear();
}
inline double Gnss::pos_cov(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.pos_cov)
  return pos_cov_.Get(index);
}
inline void Gnss::set_pos_cov(int index, double value) {
  pos_cov_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.pos_cov)
}
inline void Gnss::add_pos_cov(double value) {
  pos_cov_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.pos_cov)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::pos_cov() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.pos_cov)
  return pos_cov_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_pos_cov() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.pos_cov)
  return &pos_cov_;
}

// repeated double orien = 10;
inline int Gnss::orien_size() const {
  return orien_.size();
}
inline void Gnss::clear_orien() {
  orien_.Clear();
}
inline double Gnss::orien(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.orien)
  return orien_.Get(index);
}
inline void Gnss::set_orien(int index, double value) {
  orien_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.orien)
}
inline void Gnss::add_orien(double value) {
  orien_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.orien)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::orien() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.orien)
  return orien_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_orien() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.orien)
  return &orien_;
}

// repeated double oeirn_cov = 11;
inline int Gnss::oeirn_cov_size() const {
  return oeirn_cov_.size();
}
inline void Gnss::clear_oeirn_cov() {
  oeirn_cov_.Clear();
}
inline double Gnss::oeirn_cov(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.oeirn_cov)
  return oeirn_cov_.Get(index);
}
inline void Gnss::set_oeirn_cov(int index, double value) {
  oeirn_cov_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.oeirn_cov)
}
inline void Gnss::add_oeirn_cov(double value) {
  oeirn_cov_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.oeirn_cov)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::oeirn_cov() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.oeirn_cov)
  return oeirn_cov_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_oeirn_cov() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.oeirn_cov)
  return &oeirn_cov_;
}

// repeated double linear_vel = 12;
inline int Gnss::linear_vel_size() const {
  return linear_vel_.size();
}
inline void Gnss::clear_linear_vel() {
  linear_vel_.Clear();
}
inline double Gnss::linear_vel(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.linear_vel)
  return linear_vel_.Get(index);
}
inline void Gnss::set_linear_vel(int index, double value) {
  linear_vel_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.linear_vel)
}
inline void Gnss::add_linear_vel(double value) {
  linear_vel_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.linear_vel)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::linear_vel() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.linear_vel)
  return linear_vel_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_linear_vel() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.linear_vel)
  return &linear_vel_;
}

// repeated double linear_vel_cov = 13;
inline int Gnss::linear_vel_cov_size() const {
  return linear_vel_cov_.size();
}
inline void Gnss::clear_linear_vel_cov() {
  linear_vel_cov_.Clear();
}
inline double Gnss::linear_vel_cov(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.Gnss.linear_vel_cov)
  return linear_vel_cov_.Get(index);
}
inline void Gnss::set_linear_vel_cov(int index, double value) {
  linear_vel_cov_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.Gnss.linear_vel_cov)
}
inline void Gnss::add_linear_vel_cov(double value) {
  linear_vel_cov_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.Gnss.linear_vel_cov)
}
inline const ::google::protobuf::RepeatedField< double >&
Gnss::linear_vel_cov() const {
  // @@protoc_insertion_point(field_list:Proto_msg.Gnss.linear_vel_cov)
  return linear_vel_cov_;
}
inline ::google::protobuf::RepeatedField< double >*
Gnss::mutable_linear_vel_cov() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.Gnss.linear_vel_cov)
  return &linear_vel_cov_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace Proto_msg

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Proto_5fmsg_2eGnss_2eproto__INCLUDED
