// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Proto_msg.LidarScan.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Proto_msg.LidarScan.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace Proto_msg {
class LidarScanDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<LidarScan>
     _instance;
} _LidarScan_default_instance_;

namespace protobuf_Proto_5fmsg_2eLidarScan_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarScan, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarScan, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarScan, timestamp_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarScan, seq_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LidarScan, data_),
  0,
  1,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(LidarScan)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_LidarScan_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "Proto_msg.LidarScan.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _LidarScan_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_LidarScan_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\031Proto_msg.LidarScan.proto\022\tProto_msg\"9"
      "\n\tLidarScan\022\021\n\ttimestamp\030\001 \001(\001\022\013\n\003seq\030\002 "
      "\001(\r\022\014\n\004data\030\003 \003(\014"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 97);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Proto_msg.LidarScan.proto", &protobuf_RegisterTypes);
}
} // anonymous namespace

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_Proto_5fmsg_2eLidarScan_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LidarScan::kTimestampFieldNumber;
const int LidarScan::kSeqFieldNumber;
const int LidarScan::kDataFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LidarScan::LidarScan()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_Proto_5fmsg_2eLidarScan_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:Proto_msg.LidarScan)
}
LidarScan::LidarScan(const LidarScan& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&timestamp_, &from.timestamp_,
    static_cast<size_t>(reinterpret_cast<char*>(&seq_) -
    reinterpret_cast<char*>(&timestamp_)) + sizeof(seq_));
  // @@protoc_insertion_point(copy_constructor:Proto_msg.LidarScan)
}

void LidarScan::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&timestamp_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&seq_) -
      reinterpret_cast<char*>(&timestamp_)) + sizeof(seq_));
}

LidarScan::~LidarScan() {
  // @@protoc_insertion_point(destructor:Proto_msg.LidarScan)
  SharedDtor();
}

void LidarScan::SharedDtor() {
}

void LidarScan::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LidarScan::descriptor() {
  protobuf_Proto_5fmsg_2eLidarScan_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_Proto_5fmsg_2eLidarScan_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LidarScan& LidarScan::default_instance() {
  protobuf_Proto_5fmsg_2eLidarScan_2eproto::InitDefaults();
  return *internal_default_instance();
}

LidarScan* LidarScan::New(::google::protobuf::Arena* arena) const {
  LidarScan* n = new LidarScan;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void LidarScan::Clear() {
// @@protoc_insertion_point(message_clear_start:Proto_msg.LidarScan)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    ::memset(&timestamp_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&seq_) -
        reinterpret_cast<char*>(&timestamp_)) + sizeof(seq_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool LidarScan::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:Proto_msg.LidarScan)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double timestamp = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 seq = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_seq();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &seq_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated bytes data = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadBytes(
                input, this->add_data()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:Proto_msg.LidarScan)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:Proto_msg.LidarScan)
  return false;
#undef DO_
}

void LidarScan::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:Proto_msg.LidarScan)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double timestamp = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->timestamp(), output);
  }

  // optional uint32 seq = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->seq(), output);
  }

  // repeated bytes data = 3;
  for (int i = 0, n = this->data_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteBytes(
      3, this->data(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:Proto_msg.LidarScan)
}

::google::protobuf::uint8* LidarScan::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:Proto_msg.LidarScan)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double timestamp = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->timestamp(), target);
  }

  // optional uint32 seq = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->seq(), target);
  }

  // repeated bytes data = 3;
  for (int i = 0, n = this->data_size(); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteBytesToArray(3, this->data(i), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Proto_msg.LidarScan)
  return target;
}

size_t LidarScan::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Proto_msg.LidarScan)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated bytes data = 3;
  total_size += 1 *
      ::google::protobuf::internal::FromIntSize(this->data_size());
  for (int i = 0, n = this->data_size(); i < n; i++) {
    total_size += ::google::protobuf::internal::WireFormatLite::BytesSize(
      this->data(i));
  }

  if (_has_bits_[0 / 32] & 3u) {
    // optional double timestamp = 1;
    if (has_timestamp()) {
      total_size += 1 + 8;
    }

    // optional uint32 seq = 2;
    if (has_seq()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->seq());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LidarScan::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:Proto_msg.LidarScan)
  GOOGLE_DCHECK_NE(&from, this);
  const LidarScan* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LidarScan>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:Proto_msg.LidarScan)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:Proto_msg.LidarScan)
    MergeFrom(*source);
  }
}

void LidarScan::MergeFrom(const LidarScan& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Proto_msg.LidarScan)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      timestamp_ = from.timestamp_;
    }
    if (cached_has_bits & 0x00000002u) {
      seq_ = from.seq_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void LidarScan::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:Proto_msg.LidarScan)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LidarScan::CopyFrom(const LidarScan& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Proto_msg.LidarScan)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LidarScan::IsInitialized() const {
  return true;
}

void LidarScan::Swap(LidarScan* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LidarScan::InternalSwap(LidarScan* other) {
  using std::swap;
  data_.InternalSwap(&other->data_);
  swap(timestamp_, other->timestamp_);
  swap(seq_, other->seq_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata LidarScan::GetMetadata() const {
  protobuf_Proto_5fmsg_2eLidarScan_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_Proto_5fmsg_2eLidarScan_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// LidarScan

// optional double timestamp = 1;
bool LidarScan::has_timestamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void LidarScan::set_has_timestamp() {
  _has_bits_[0] |= 0x00000001u;
}
void LidarScan::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000001u;
}
void LidarScan::clear_timestamp() {
  timestamp_ = 0;
  clear_has_timestamp();
}
double LidarScan::timestamp() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarScan.timestamp)
  return timestamp_;
}
void LidarScan::set_timestamp(double value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarScan.timestamp)
}

// optional uint32 seq = 2;
bool LidarScan::has_seq() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void LidarScan::set_has_seq() {
  _has_bits_[0] |= 0x00000002u;
}
void LidarScan::clear_has_seq() {
  _has_bits_[0] &= ~0x00000002u;
}
void LidarScan::clear_seq() {
  seq_ = 0u;
  clear_has_seq();
}
::google::protobuf::uint32 LidarScan::seq() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarScan.seq)
  return seq_;
}
void LidarScan::set_seq(::google::protobuf::uint32 value) {
  set_has_seq();
  seq_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarScan.seq)
}

// repeated bytes data = 3;
int LidarScan::data_size() const {
  return data_.size();
}
void LidarScan::clear_data() {
  data_.Clear();
}
const ::std::string& LidarScan::data(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarScan.data)
  return data_.Get(index);
}
::std::string* LidarScan::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:Proto_msg.LidarScan.data)
  return data_.Mutable(index);
}
void LidarScan::set_data(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:Proto_msg.LidarScan.data)
  data_.Mutable(index)->assign(value);
}
#if LANG_CXX11
void LidarScan::set_data(int index, ::std::string&& value) {
  // @@protoc_insertion_point(field_set:Proto_msg.LidarScan.data)
  data_.Mutable(index)->assign(std::move(value));
}
#endif
void LidarScan::set_data(int index, const char* value) {
  GOOGLE_DCHECK(value != NULL);
  data_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:Proto_msg.LidarScan.data)
}
void LidarScan::set_data(int index, const void* value, size_t size) {
  data_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.LidarScan.data)
}
::std::string* LidarScan::add_data() {
  // @@protoc_insertion_point(field_add_mutable:Proto_msg.LidarScan.data)
  return data_.Add();
}
void LidarScan::add_data(const ::std::string& value) {
  data_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:Proto_msg.LidarScan.data)
}
#if LANG_CXX11
void LidarScan::add_data(::std::string&& value) {
  data_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:Proto_msg.LidarScan.data)
}
#endif
void LidarScan::add_data(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  data_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:Proto_msg.LidarScan.data)
}
void LidarScan::add_data(const void* value, size_t size) {
  data_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:Proto_msg.LidarScan.data)
}
const ::google::protobuf::RepeatedPtrField< ::std::string>&
LidarScan::data() const {
  // @@protoc_insertion_point(field_list:Proto_msg.LidarScan.data)
  return data_;
}
::google::protobuf::RepeatedPtrField< ::std::string>*
LidarScan::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.LidarScan.data)
  return &data_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace Proto_msg

// @@protoc_insertion_point(global_scope)
