// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Proto_msg.Imu.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Proto_msg.Imu.pb.h"

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

const ::google::protobuf::Descriptor* Imu_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Imu_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Proto_5fmsg_2eImu_2eproto() {
  protobuf_AddDesc_Proto_5fmsg_2eImu_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Proto_msg.Imu.proto");
  GOOGLE_CHECK(file != NULL);
  Imu_descriptor_ = file->message_type(0);
  static const int Imu_offsets_[12] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, seq_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, parent_frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, frame_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, orien_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, orien_cov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, angular_vel_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, angular_vel_cov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, angular_bias_cov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, acc_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, acc_cov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, acc_bias_cov_),
  };
  Imu_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Imu_descriptor_,
      Imu::default_instance_,
      Imu_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Imu));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Proto_5fmsg_2eImu_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Imu_descriptor_, &Imu::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Proto_5fmsg_2eImu_2eproto() {
  delete Imu::default_instance_;
  delete Imu_reflection_;
}

void protobuf_AddDesc_Proto_5fmsg_2eImu_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\023Proto_msg.Imu.proto\022\tProto_msg\"\356\001\n\003Imu"
    "\022\021\n\ttimestamp\030\001 \001(\001\022\013\n\003seq\030\002 \001(\r\022\027\n\017pare"
    "nt_frame_id\030\003 \001(\t\022\020\n\010frame_id\030\004 \001(\t\022\r\n\005o"
    "rien\030\005 \003(\001\022\021\n\torien_cov\030\006 \003(\001\022\023\n\013angular"
    "_vel\030\007 \003(\001\022\027\n\017angular_vel_cov\030\010 \003(\001\022\030\n\020a"
    "ngular_bias_cov\030\t \003(\001\022\013\n\003acc\030\n \003(\001\022\017\n\007ac"
    "c_cov\030\013 \003(\001\022\024\n\014acc_bias_cov\030\014 \003(\001", 273);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Proto_msg.Imu.proto", &protobuf_RegisterTypes);
  Imu::default_instance_ = new Imu();
  Imu::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Proto_5fmsg_2eImu_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Proto_5fmsg_2eImu_2eproto {
  StaticDescriptorInitializer_Proto_5fmsg_2eImu_2eproto() {
    protobuf_AddDesc_Proto_5fmsg_2eImu_2eproto();
  }
} static_descriptor_initializer_Proto_5fmsg_2eImu_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Imu::kTimestampFieldNumber;
const int Imu::kSeqFieldNumber;
const int Imu::kParentFrameIdFieldNumber;
const int Imu::kFrameIdFieldNumber;
const int Imu::kOrienFieldNumber;
const int Imu::kOrienCovFieldNumber;
const int Imu::kAngularVelFieldNumber;
const int Imu::kAngularVelCovFieldNumber;
const int Imu::kAngularBiasCovFieldNumber;
const int Imu::kAccFieldNumber;
const int Imu::kAccCovFieldNumber;
const int Imu::kAccBiasCovFieldNumber;
#endif  // !_MSC_VER

Imu::Imu()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:Proto_msg.Imu)
}

void Imu::InitAsDefaultInstance() {
}

Imu::Imu(const Imu& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:Proto_msg.Imu)
}

void Imu::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  timestamp_ = 0;
  seq_ = 0u;
  parent_frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:Proto_msg.Imu)
  SharedDtor();
}

void Imu::SharedDtor() {
  if (parent_frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete parent_frame_id_;
  }
  if (frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete frame_id_;
  }
  if (this != default_instance_) {
  }
}

void Imu::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Imu::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Imu_descriptor_;
}

const Imu& Imu::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Proto_5fmsg_2eImu_2eproto();
  return *default_instance_;
}

Imu* Imu::default_instance_ = NULL;

Imu* Imu::New() const {
  return new Imu;
}

void Imu::Clear() {
  if (_has_bits_[0 / 32] & 15) {
    timestamp_ = 0;
    seq_ = 0u;
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
  orien_.Clear();
  orien_cov_.Clear();
  angular_vel_.Clear();
  angular_vel_cov_.Clear();
  angular_bias_cov_.Clear();
  acc_.Clear();
  acc_cov_.Clear();
  acc_bias_cov_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Imu::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:Proto_msg.Imu)
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
        if (input->ExpectTag(41)) goto parse_orien;
        break;
      }

      // repeated double orien = 5;
      case 5: {
        if (tag == 41) {
         parse_orien:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 41, input, this->mutable_orien())));
        } else if (tag == 42) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_orien())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(41)) goto parse_orien;
        if (input->ExpectTag(49)) goto parse_orien_cov;
        break;
      }

      // repeated double orien_cov = 6;
      case 6: {
        if (tag == 49) {
         parse_orien_cov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 49, input, this->mutable_orien_cov())));
        } else if (tag == 50) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_orien_cov())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(49)) goto parse_orien_cov;
        if (input->ExpectTag(57)) goto parse_angular_vel;
        break;
      }

      // repeated double angular_vel = 7;
      case 7: {
        if (tag == 57) {
         parse_angular_vel:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 57, input, this->mutable_angular_vel())));
        } else if (tag == 58) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_angular_vel())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(57)) goto parse_angular_vel;
        if (input->ExpectTag(65)) goto parse_angular_vel_cov;
        break;
      }

      // repeated double angular_vel_cov = 8;
      case 8: {
        if (tag == 65) {
         parse_angular_vel_cov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 65, input, this->mutable_angular_vel_cov())));
        } else if (tag == 66) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_angular_vel_cov())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(65)) goto parse_angular_vel_cov;
        if (input->ExpectTag(73)) goto parse_angular_bias_cov;
        break;
      }

      // repeated double angular_bias_cov = 9;
      case 9: {
        if (tag == 73) {
         parse_angular_bias_cov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 73, input, this->mutable_angular_bias_cov())));
        } else if (tag == 74) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_angular_bias_cov())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(73)) goto parse_angular_bias_cov;
        if (input->ExpectTag(81)) goto parse_acc;
        break;
      }

      // repeated double acc = 10;
      case 10: {
        if (tag == 81) {
         parse_acc:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 81, input, this->mutable_acc())));
        } else if (tag == 82) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_acc())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(81)) goto parse_acc;
        if (input->ExpectTag(89)) goto parse_acc_cov;
        break;
      }

      // repeated double acc_cov = 11;
      case 11: {
        if (tag == 89) {
         parse_acc_cov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 89, input, this->mutable_acc_cov())));
        } else if (tag == 90) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_acc_cov())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(89)) goto parse_acc_cov;
        if (input->ExpectTag(97)) goto parse_acc_bias_cov;
        break;
      }

      // repeated double acc_bias_cov = 12;
      case 12: {
        if (tag == 97) {
         parse_acc_bias_cov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 97, input, this->mutable_acc_bias_cov())));
        } else if (tag == 98) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_acc_bias_cov())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(97)) goto parse_acc_bias_cov;
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
  // @@protoc_insertion_point(parse_success:Proto_msg.Imu)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:Proto_msg.Imu)
  return false;
#undef DO_
}

void Imu::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:Proto_msg.Imu)
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

  // repeated double orien = 5;
  for (int i = 0; i < this->orien_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      5, this->orien(i), output);
  }

  // repeated double orien_cov = 6;
  for (int i = 0; i < this->orien_cov_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      6, this->orien_cov(i), output);
  }

  // repeated double angular_vel = 7;
  for (int i = 0; i < this->angular_vel_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      7, this->angular_vel(i), output);
  }

  // repeated double angular_vel_cov = 8;
  for (int i = 0; i < this->angular_vel_cov_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      8, this->angular_vel_cov(i), output);
  }

  // repeated double angular_bias_cov = 9;
  for (int i = 0; i < this->angular_bias_cov_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      9, this->angular_bias_cov(i), output);
  }

  // repeated double acc = 10;
  for (int i = 0; i < this->acc_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      10, this->acc(i), output);
  }

  // repeated double acc_cov = 11;
  for (int i = 0; i < this->acc_cov_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      11, this->acc_cov(i), output);
  }

  // repeated double acc_bias_cov = 12;
  for (int i = 0; i < this->acc_bias_cov_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      12, this->acc_bias_cov(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:Proto_msg.Imu)
}

::google::protobuf::uint8* Imu::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:Proto_msg.Imu)
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

  // repeated double orien = 5;
  for (int i = 0; i < this->orien_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(5, this->orien(i), target);
  }

  // repeated double orien_cov = 6;
  for (int i = 0; i < this->orien_cov_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(6, this->orien_cov(i), target);
  }

  // repeated double angular_vel = 7;
  for (int i = 0; i < this->angular_vel_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(7, this->angular_vel(i), target);
  }

  // repeated double angular_vel_cov = 8;
  for (int i = 0; i < this->angular_vel_cov_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(8, this->angular_vel_cov(i), target);
  }

  // repeated double angular_bias_cov = 9;
  for (int i = 0; i < this->angular_bias_cov_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(9, this->angular_bias_cov(i), target);
  }

  // repeated double acc = 10;
  for (int i = 0; i < this->acc_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(10, this->acc(i), target);
  }

  // repeated double acc_cov = 11;
  for (int i = 0; i < this->acc_cov_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(11, this->acc_cov(i), target);
  }

  // repeated double acc_bias_cov = 12;
  for (int i = 0; i < this->acc_bias_cov_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(12, this->acc_bias_cov(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Proto_msg.Imu)
  return target;
}

int Imu::ByteSize() const {
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

  }
  // repeated double orien = 5;
  {
    int data_size = 0;
    data_size = 8 * this->orien_size();
    total_size += 1 * this->orien_size() + data_size;
  }

  // repeated double orien_cov = 6;
  {
    int data_size = 0;
    data_size = 8 * this->orien_cov_size();
    total_size += 1 * this->orien_cov_size() + data_size;
  }

  // repeated double angular_vel = 7;
  {
    int data_size = 0;
    data_size = 8 * this->angular_vel_size();
    total_size += 1 * this->angular_vel_size() + data_size;
  }

  // repeated double angular_vel_cov = 8;
  {
    int data_size = 0;
    data_size = 8 * this->angular_vel_cov_size();
    total_size += 1 * this->angular_vel_cov_size() + data_size;
  }

  // repeated double angular_bias_cov = 9;
  {
    int data_size = 0;
    data_size = 8 * this->angular_bias_cov_size();
    total_size += 1 * this->angular_bias_cov_size() + data_size;
  }

  // repeated double acc = 10;
  {
    int data_size = 0;
    data_size = 8 * this->acc_size();
    total_size += 1 * this->acc_size() + data_size;
  }

  // repeated double acc_cov = 11;
  {
    int data_size = 0;
    data_size = 8 * this->acc_cov_size();
    total_size += 1 * this->acc_cov_size() + data_size;
  }

  // repeated double acc_bias_cov = 12;
  {
    int data_size = 0;
    data_size = 8 * this->acc_bias_cov_size();
    total_size += 1 * this->acc_bias_cov_size() + data_size;
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

void Imu::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Imu* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Imu*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Imu::MergeFrom(const Imu& from) {
  GOOGLE_CHECK_NE(&from, this);
  orien_.MergeFrom(from.orien_);
  orien_cov_.MergeFrom(from.orien_cov_);
  angular_vel_.MergeFrom(from.angular_vel_);
  angular_vel_cov_.MergeFrom(from.angular_vel_cov_);
  angular_bias_cov_.MergeFrom(from.angular_bias_cov_);
  acc_.MergeFrom(from.acc_);
  acc_cov_.MergeFrom(from.acc_cov_);
  acc_bias_cov_.MergeFrom(from.acc_bias_cov_);
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
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Imu::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Imu::CopyFrom(const Imu& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {

  return true;
}

void Imu::Swap(Imu* other) {
  if (other != this) {
    std::swap(timestamp_, other->timestamp_);
    std::swap(seq_, other->seq_);
    std::swap(parent_frame_id_, other->parent_frame_id_);
    std::swap(frame_id_, other->frame_id_);
    orien_.Swap(&other->orien_);
    orien_cov_.Swap(&other->orien_cov_);
    angular_vel_.Swap(&other->angular_vel_);
    angular_vel_cov_.Swap(&other->angular_vel_cov_);
    angular_bias_cov_.Swap(&other->angular_bias_cov_);
    acc_.Swap(&other->acc_);
    acc_cov_.Swap(&other->acc_cov_);
    acc_bias_cov_.Swap(&other->acc_bias_cov_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Imu::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Imu_descriptor_;
  metadata.reflection = Imu_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace Proto_msg

// @@protoc_insertion_point(global_scope)
