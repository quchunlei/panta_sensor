// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Proto_msg.GridMap.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Proto_msg.GridMap.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace Proto_msg {

namespace {

const ::google::protobuf::Descriptor* GridMap_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  GridMap_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Proto_5fmsg_2eGridMap_2eproto() {
  protobuf_AddDesc_Proto_5fmsg_2eGridMap_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Proto_msg.GridMap.proto");
  GOOGLE_CHECK(file != NULL);
  GridMap_descriptor_ = file->message_type(0);
  static const int GridMap_offsets_[10] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, seq_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, parent_frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, width_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, height_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, resolution_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, origin_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, orientation_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, grids_),
  };
  GridMap_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      GridMap_descriptor_,
      GridMap::default_instance_,
      GridMap_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(GridMap, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(GridMap));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Proto_5fmsg_2eGridMap_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    GridMap_descriptor_, &GridMap::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Proto_5fmsg_2eGridMap_2eproto() {
  delete GridMap::default_instance_;
  delete GridMap_reflection_;
}

void protobuf_AddDesc_Proto_5fmsg_2eGridMap_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\027Proto_msg.GridMap.proto\022\tProto_msg\"\273\001\n"
    "\007GridMap\022\021\n\ttimestamp\030\001 \001(\001\022\013\n\003seq\030\002 \001(\r"
    "\022\027\n\017parent_frame_id\030\003 \001(\t\022\020\n\010frame_id\030\004 "
    "\001(\t\022\r\n\005width\030\005 \001(\r\022\016\n\006height\030\006 \001(\r\022\022\n\nre"
    "solution\030\007 \001(\002\022\016\n\006origin\030\010 \003(\002\022\023\n\013orient"
    "ation\030\t \003(\002\022\r\n\005grids\030\n \003(\002", 226);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Proto_msg.GridMap.proto", &protobuf_RegisterTypes);
  GridMap::default_instance_ = new GridMap();
  GridMap::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Proto_5fmsg_2eGridMap_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Proto_5fmsg_2eGridMap_2eproto {
  StaticDescriptorInitializer_Proto_5fmsg_2eGridMap_2eproto() {
    protobuf_AddDesc_Proto_5fmsg_2eGridMap_2eproto();
  }
} static_descriptor_initializer_Proto_5fmsg_2eGridMap_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int GridMap::kTimestampFieldNumber;
const int GridMap::kSeqFieldNumber;
const int GridMap::kParentFrameIdFieldNumber;
const int GridMap::kFrameIdFieldNumber;
const int GridMap::kWidthFieldNumber;
const int GridMap::kHeightFieldNumber;
const int GridMap::kResolutionFieldNumber;
const int GridMap::kOriginFieldNumber;
const int GridMap::kOrientationFieldNumber;
const int GridMap::kGridsFieldNumber;
#endif  // !_MSC_VER

GridMap::GridMap()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Proto_msg.GridMap)
}

void GridMap::InitAsDefaultInstance() {
}

GridMap::GridMap(const GridMap& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:Proto_msg.GridMap)
}

void GridMap::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  timestamp_ = 0;
  seq_ = 0u;
  parent_frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  width_ = 0u;
  height_ = 0u;
  resolution_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

GridMap::~GridMap() {
  // @@protoc_insertion_point(destructor:Proto_msg.GridMap)
  SharedDtor();
}

void GridMap::SharedDtor() {
  if (parent_frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete parent_frame_id_;
  }
  if (frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete frame_id_;
  }
  if (this != default_instance_) {
  }
}

void GridMap::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* GridMap::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return GridMap_descriptor_;
}

const GridMap& GridMap::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Proto_5fmsg_2eGridMap_2eproto();
  return *default_instance_;
}

GridMap* GridMap::default_instance_ = NULL;

GridMap* GridMap::New() const {
  return new GridMap;
}

void GridMap::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<GridMap*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  if (_has_bits_[0 / 32] & 127) {
    ZR_(seq_, width_);
    ZR_(height_, resolution_);
    timestamp_ = 0;
    if (has_parent_frame_id()) {
      if (parent_frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
        parent_frame_id_->clear();
      }
    }
    if (has_frame_id()) {
      if (frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
        frame_id_->clear();
      }
    }
  }

#undef OFFSET_OF_FIELD_
#undef ZR_

  origin_.Clear();
  orientation_.Clear();
  grids_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool GridMap::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:Proto_msg.GridMap)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double timestamp = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_)));
          set_has_timestamp();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_seq;
        break;
      }

      // optional uint32 seq = 2;
      case 2: {
        if (tag == 16) {
         parse_seq:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &seq_)));
          set_has_seq();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_parent_frame_id;
        break;
      }

      // optional string parent_frame_id = 3;
      case 3: {
        if (tag == 26) {
         parse_parent_frame_id:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_parent_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->parent_frame_id().data(), this->parent_frame_id().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "parent_frame_id");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_frame_id;
        break;
      }

      // optional string frame_id = 4;
      case 4: {
        if (tag == 34) {
         parse_frame_id:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->frame_id().data(), this->frame_id().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "frame_id");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(40)) goto parse_width;
        break;
      }

      // optional uint32 width = 5;
      case 5: {
        if (tag == 40) {
         parse_width:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &width_)));
          set_has_width();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(48)) goto parse_height;
        break;
      }

      // optional uint32 height = 6;
      case 6: {
        if (tag == 48) {
         parse_height:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &height_)));
          set_has_height();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(61)) goto parse_resolution;
        break;
      }

      // optional float resolution = 7;
      case 7: {
        if (tag == 61) {
         parse_resolution:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &resolution_)));
          set_has_resolution();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(69)) goto parse_origin;
        break;
      }

      // repeated float origin = 8;
      case 8: {
        if (tag == 69) {
         parse_origin:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 69, input, this->mutable_origin())));
        } else if (tag == 66) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_origin())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(69)) goto parse_origin;
        if (input->ExpectTag(77)) goto parse_orientation;
        break;
      }

      // repeated float orientation = 9;
      case 9: {
        if (tag == 77) {
         parse_orientation:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 77, input, this->mutable_orientation())));
        } else if (tag == 74) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_orientation())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(77)) goto parse_orientation;
        if (input->ExpectTag(85)) goto parse_grids;
        break;
      }

      // repeated float grids = 10;
      case 10: {
        if (tag == 85) {
         parse_grids:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 85, input, this->mutable_grids())));
        } else if (tag == 82) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_grids())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(85)) goto parse_grids;
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:Proto_msg.GridMap)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:Proto_msg.GridMap)
  return false;
#undef DO_
}

void GridMap::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:Proto_msg.GridMap)
  // optional double timestamp = 1;
  if (has_timestamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->timestamp(), output);
  }

  // optional uint32 seq = 2;
  if (has_seq()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->seq(), output);
  }

  // optional string parent_frame_id = 3;
  if (has_parent_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parent_frame_id().data(), this->parent_frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "parent_frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->parent_frame_id(), output);
  }

  // optional string frame_id = 4;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      4, this->frame_id(), output);
  }

  // optional uint32 width = 5;
  if (has_width()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(5, this->width(), output);
  }

  // optional uint32 height = 6;
  if (has_height()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(6, this->height(), output);
  }

  // optional float resolution = 7;
  if (has_resolution()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(7, this->resolution(), output);
  }

  // repeated float origin = 8;
  for (int i = 0; i < this->origin_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(
      8, this->origin(i), output);
  }

  // repeated float orientation = 9;
  for (int i = 0; i < this->orientation_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(
      9, this->orientation(i), output);
  }

  // repeated float grids = 10;
  for (int i = 0; i < this->grids_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(
      10, this->grids(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:Proto_msg.GridMap)
}

::google::protobuf::uint8* GridMap::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:Proto_msg.GridMap)
  // optional double timestamp = 1;
  if (has_timestamp()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->timestamp(), target);
  }

  // optional uint32 seq = 2;
  if (has_seq()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->seq(), target);
  }

  // optional string parent_frame_id = 3;
  if (has_parent_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parent_frame_id().data(), this->parent_frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "parent_frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->parent_frame_id(), target);
  }

  // optional string frame_id = 4;
  if (has_frame_id()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), this->frame_id().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        4, this->frame_id(), target);
  }

  // optional uint32 width = 5;
  if (has_width()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(5, this->width(), target);
  }

  // optional uint32 height = 6;
  if (has_height()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(6, this->height(), target);
  }

  // optional float resolution = 7;
  if (has_resolution()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(7, this->resolution(), target);
  }

  // repeated float origin = 8;
  for (int i = 0; i < this->origin_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatToArray(8, this->origin(i), target);
  }

  // repeated float orientation = 9;
  for (int i = 0; i < this->orientation_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatToArray(9, this->orientation(i), target);
  }

  // repeated float grids = 10;
  for (int i = 0; i < this->grids_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatToArray(10, this->grids(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Proto_msg.GridMap)
  return target;
}

int GridMap::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
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

    // optional string parent_frame_id = 3;
    if (has_parent_frame_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->parent_frame_id());
    }

    // optional string frame_id = 4;
    if (has_frame_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->frame_id());
    }

    // optional uint32 width = 5;
    if (has_width()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->width());
    }

    // optional uint32 height = 6;
    if (has_height()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->height());
    }

    // optional float resolution = 7;
    if (has_resolution()) {
      total_size += 1 + 4;
    }

  }
  // repeated float origin = 8;
  {
    int data_size = 0;
    data_size = 4 * this->origin_size();
    total_size += 1 * this->origin_size() + data_size;
  }

  // repeated float orientation = 9;
  {
    int data_size = 0;
    data_size = 4 * this->orientation_size();
    total_size += 1 * this->orientation_size() + data_size;
  }

  // repeated float grids = 10;
  {
    int data_size = 0;
    data_size = 4 * this->grids_size();
    total_size += 1 * this->grids_size() + data_size;
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void GridMap::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const GridMap* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const GridMap*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void GridMap::MergeFrom(const GridMap& from) {
  GOOGLE_CHECK_NE(&from, this);
  origin_.MergeFrom(from.origin_);
  orientation_.MergeFrom(from.orientation_);
  grids_.MergeFrom(from.grids_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_timestamp()) {
      set_timestamp(from.timestamp());
    }
    if (from.has_seq()) {
      set_seq(from.seq());
    }
    if (from.has_parent_frame_id()) {
      set_parent_frame_id(from.parent_frame_id());
    }
    if (from.has_frame_id()) {
      set_frame_id(from.frame_id());
    }
    if (from.has_width()) {
      set_width(from.width());
    }
    if (from.has_height()) {
      set_height(from.height());
    }
    if (from.has_resolution()) {
      set_resolution(from.resolution());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void GridMap::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void GridMap::CopyFrom(const GridMap& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GridMap::IsInitialized() const {

  return true;
}

void GridMap::Swap(GridMap* other) {
  if (other != this) {
    std::swap(timestamp_, other->timestamp_);
    std::swap(seq_, other->seq_);
    std::swap(parent_frame_id_, other->parent_frame_id_);
    std::swap(frame_id_, other->frame_id_);
    std::swap(width_, other->width_);
    std::swap(height_, other->height_);
    std::swap(resolution_, other->resolution_);
    origin_.Swap(&other->origin_);
    orientation_.Swap(&other->orientation_);
    grids_.Swap(&other->grids_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata GridMap::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = GridMap_descriptor_;
  metadata.reflection = GridMap_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace Proto_msg

// @@protoc_insertion_point(global_scope)
