// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Proto_msg.LidarPoints.proto

#ifndef PROTOBUF_Proto_5fmsg_2eLidarPoints_2eproto__INCLUDED
#define PROTOBUF_Proto_5fmsg_2eLidarPoints_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace Proto_msg {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Proto_5fmsg_2eLidarPoints_2eproto();
void protobuf_AssignDesc_Proto_5fmsg_2eLidarPoints_2eproto();
void protobuf_ShutdownFile_Proto_5fmsg_2eLidarPoints_2eproto();

class LidarPoints;

// ===================================================================

class LidarPoints : public ::google::protobuf::Message {
 public:
  LidarPoints();
  virtual ~LidarPoints();

  LidarPoints(const LidarPoints& from);

  inline LidarPoints& operator=(const LidarPoints& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const LidarPoints& default_instance();

  void Swap(LidarPoints* other);

  // implements Message ----------------------------------------------

  LidarPoints* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LidarPoints& from);
  void MergeFrom(const LidarPoints& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double timestamp = 1;
  inline bool has_timestamp() const;
  inline void clear_timestamp();
  static const int kTimestampFieldNumber = 1;
  inline double timestamp() const;
  inline void set_timestamp(double value);

  // optional uint32 seq = 2;
  inline bool has_seq() const;
  inline void clear_seq();
  static const int kSeqFieldNumber = 2;
  inline ::google::protobuf::uint32 seq() const;
  inline void set_seq(::google::protobuf::uint32 value);

  // optional string parent_frame_id = 3;
  inline bool has_parent_frame_id() const;
  inline void clear_parent_frame_id();
  static const int kParentFrameIdFieldNumber = 3;
  inline const ::std::string& parent_frame_id() const;
  inline void set_parent_frame_id(const ::std::string& value);
  inline void set_parent_frame_id(const char* value);
  inline void set_parent_frame_id(const char* value, size_t size);
  inline ::std::string* mutable_parent_frame_id();
  inline ::std::string* release_parent_frame_id();
  inline void set_allocated_parent_frame_id(::std::string* parent_frame_id);

  // optional string frame_id = 4;
  inline bool has_frame_id() const;
  inline void clear_frame_id();
  static const int kFrameIdFieldNumber = 4;
  inline const ::std::string& frame_id() const;
  inline void set_frame_id(const ::std::string& value);
  inline void set_frame_id(const char* value);
  inline void set_frame_id(const char* value, size_t size);
  inline ::std::string* mutable_frame_id();
  inline ::std::string* release_frame_id();
  inline void set_allocated_frame_id(::std::string* frame_id);

  // optional bool motion_correct = 5;
  inline bool has_motion_correct() const;
  inline void clear_motion_correct();
  static const int kMotionCorrectFieldNumber = 5;
  inline bool motion_correct() const;
  inline void set_motion_correct(bool value);

  // optional uint32 height = 6;
  inline bool has_height() const;
  inline void clear_height();
  static const int kHeightFieldNumber = 6;
  inline ::google::protobuf::uint32 height() const;
  inline void set_height(::google::protobuf::uint32 value);

  // optional uint32 width = 7;
  inline bool has_width() const;
  inline void clear_width();
  static const int kWidthFieldNumber = 7;
  inline ::google::protobuf::uint32 width() const;
  inline void set_width(::google::protobuf::uint32 value);

  // optional bool is_dense = 8;
  inline bool has_is_dense() const;
  inline void clear_is_dense();
  static const int kIsDenseFieldNumber = 8;
  inline bool is_dense() const;
  inline void set_is_dense(bool value);

  // optional bool is_transform = 9;
  inline bool has_is_transform() const;
  inline void clear_is_transform();
  static const int kIsTransformFieldNumber = 9;
  inline bool is_transform() const;
  inline void set_is_transform(bool value);

  // optional string lidar_model = 10;
  inline bool has_lidar_model() const;
  inline void clear_lidar_model();
  static const int kLidarModelFieldNumber = 10;
  inline const ::std::string& lidar_model() const;
  inline void set_lidar_model(const ::std::string& value);
  inline void set_lidar_model(const char* value);
  inline void set_lidar_model(const char* value, size_t size);
  inline ::std::string* mutable_lidar_model();
  inline ::std::string* release_lidar_model();
  inline void set_allocated_lidar_model(::std::string* lidar_model);

  // optional string points_type = 11;
  inline bool has_points_type() const;
  inline void clear_points_type();
  static const int kPointsTypeFieldNumber = 11;
  inline const ::std::string& points_type() const;
  inline void set_points_type(const ::std::string& value);
  inline void set_points_type(const char* value);
  inline void set_points_type(const char* value, size_t size);
  inline ::std::string* mutable_points_type();
  inline ::std::string* release_points_type();
  inline void set_allocated_points_type(::std::string* points_type);

  // repeated float data = 12;
  inline int data_size() const;
  inline void clear_data();
  static const int kDataFieldNumber = 12;
  inline float data(int index) const;
  inline void set_data(int index, float value);
  inline void add_data(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      data() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_data();

  // @@protoc_insertion_point(class_scope:Proto_msg.LidarPoints)
 private:
  inline void set_has_timestamp();
  inline void clear_has_timestamp();
  inline void set_has_seq();
  inline void clear_has_seq();
  inline void set_has_parent_frame_id();
  inline void clear_has_parent_frame_id();
  inline void set_has_frame_id();
  inline void clear_has_frame_id();
  inline void set_has_motion_correct();
  inline void clear_has_motion_correct();
  inline void set_has_height();
  inline void clear_has_height();
  inline void set_has_width();
  inline void clear_has_width();
  inline void set_has_is_dense();
  inline void clear_has_is_dense();
  inline void set_has_is_transform();
  inline void clear_has_is_transform();
  inline void set_has_lidar_model();
  inline void clear_has_lidar_model();
  inline void set_has_points_type();
  inline void clear_has_points_type();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double timestamp_;
  ::std::string* parent_frame_id_;
  ::std::string* frame_id_;
  ::google::protobuf::uint32 seq_;
  ::google::protobuf::uint32 height_;
  ::google::protobuf::uint32 width_;
  bool motion_correct_;
  bool is_dense_;
  bool is_transform_;
  ::std::string* lidar_model_;
  ::std::string* points_type_;
  ::google::protobuf::RepeatedField< float > data_;
  friend void  protobuf_AddDesc_Proto_5fmsg_2eLidarPoints_2eproto();
  friend void protobuf_AssignDesc_Proto_5fmsg_2eLidarPoints_2eproto();
  friend void protobuf_ShutdownFile_Proto_5fmsg_2eLidarPoints_2eproto();

  void InitAsDefaultInstance();
  static LidarPoints* default_instance_;
};
// ===================================================================


// ===================================================================

// LidarPoints

// optional double timestamp = 1;
inline bool LidarPoints::has_timestamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LidarPoints::set_has_timestamp() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LidarPoints::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LidarPoints::clear_timestamp() {
  timestamp_ = 0;
  clear_has_timestamp();
}
inline double LidarPoints::timestamp() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.timestamp)
  return timestamp_;
}
inline void LidarPoints::set_timestamp(double value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.timestamp)
}

// optional uint32 seq = 2;
inline bool LidarPoints::has_seq() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LidarPoints::set_has_seq() {
  _has_bits_[0] |= 0x00000002u;
}
inline void LidarPoints::clear_has_seq() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void LidarPoints::clear_seq() {
  seq_ = 0u;
  clear_has_seq();
}
inline ::google::protobuf::uint32 LidarPoints::seq() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.seq)
  return seq_;
}
inline void LidarPoints::set_seq(::google::protobuf::uint32 value) {
  set_has_seq();
  seq_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.seq)
}

// optional string parent_frame_id = 3;
inline bool LidarPoints::has_parent_frame_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LidarPoints::set_has_parent_frame_id() {
  _has_bits_[0] |= 0x00000004u;
}
inline void LidarPoints::clear_has_parent_frame_id() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void LidarPoints::clear_parent_frame_id() {
  if (parent_frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    parent_frame_id_->clear();
  }
  clear_has_parent_frame_id();
}
inline const ::std::string& LidarPoints::parent_frame_id() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.parent_frame_id)
  return *parent_frame_id_;
}
inline void LidarPoints::set_parent_frame_id(const ::std::string& value) {
  set_has_parent_frame_id();
  if (parent_frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    parent_frame_id_ = new ::std::string;
  }
  parent_frame_id_->assign(value);
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.parent_frame_id)
}
inline void LidarPoints::set_parent_frame_id(const char* value) {
  set_has_parent_frame_id();
  if (parent_frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    parent_frame_id_ = new ::std::string;
  }
  parent_frame_id_->assign(value);
  // @@protoc_insertion_point(field_set_char:Proto_msg.LidarPoints.parent_frame_id)
}
inline void LidarPoints::set_parent_frame_id(const char* value, size_t size) {
  set_has_parent_frame_id();
  if (parent_frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    parent_frame_id_ = new ::std::string;
  }
  parent_frame_id_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.LidarPoints.parent_frame_id)
}
inline ::std::string* LidarPoints::mutable_parent_frame_id() {
  set_has_parent_frame_id();
  if (parent_frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    parent_frame_id_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:Proto_msg.LidarPoints.parent_frame_id)
  return parent_frame_id_;
}
inline ::std::string* LidarPoints::release_parent_frame_id() {
  clear_has_parent_frame_id();
  if (parent_frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = parent_frame_id_;
    parent_frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void LidarPoints::set_allocated_parent_frame_id(::std::string* parent_frame_id) {
  if (parent_frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete parent_frame_id_;
  }
  if (parent_frame_id) {
    set_has_parent_frame_id();
    parent_frame_id_ = parent_frame_id;
  } else {
    clear_has_parent_frame_id();
    parent_frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.LidarPoints.parent_frame_id)
}

// optional string frame_id = 4;
inline bool LidarPoints::has_frame_id() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void LidarPoints::set_has_frame_id() {
  _has_bits_[0] |= 0x00000008u;
}
inline void LidarPoints::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void LidarPoints::clear_frame_id() {
  if (frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    frame_id_->clear();
  }
  clear_has_frame_id();
}
inline const ::std::string& LidarPoints::frame_id() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.frame_id)
  return *frame_id_;
}
inline void LidarPoints::set_frame_id(const ::std::string& value) {
  set_has_frame_id();
  if (frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    frame_id_ = new ::std::string;
  }
  frame_id_->assign(value);
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.frame_id)
}
inline void LidarPoints::set_frame_id(const char* value) {
  set_has_frame_id();
  if (frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    frame_id_ = new ::std::string;
  }
  frame_id_->assign(value);
  // @@protoc_insertion_point(field_set_char:Proto_msg.LidarPoints.frame_id)
}
inline void LidarPoints::set_frame_id(const char* value, size_t size) {
  set_has_frame_id();
  if (frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    frame_id_ = new ::std::string;
  }
  frame_id_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.LidarPoints.frame_id)
}
inline ::std::string* LidarPoints::mutable_frame_id() {
  set_has_frame_id();
  if (frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    frame_id_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:Proto_msg.LidarPoints.frame_id)
  return frame_id_;
}
inline ::std::string* LidarPoints::release_frame_id() {
  clear_has_frame_id();
  if (frame_id_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = frame_id_;
    frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void LidarPoints::set_allocated_frame_id(::std::string* frame_id) {
  if (frame_id_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete frame_id_;
  }
  if (frame_id) {
    set_has_frame_id();
    frame_id_ = frame_id;
  } else {
    clear_has_frame_id();
    frame_id_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.LidarPoints.frame_id)
}

// optional bool motion_correct = 5;
inline bool LidarPoints::has_motion_correct() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void LidarPoints::set_has_motion_correct() {
  _has_bits_[0] |= 0x00000010u;
}
inline void LidarPoints::clear_has_motion_correct() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void LidarPoints::clear_motion_correct() {
  motion_correct_ = false;
  clear_has_motion_correct();
}
inline bool LidarPoints::motion_correct() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.motion_correct)
  return motion_correct_;
}
inline void LidarPoints::set_motion_correct(bool value) {
  set_has_motion_correct();
  motion_correct_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.motion_correct)
}

// optional uint32 height = 6;
inline bool LidarPoints::has_height() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void LidarPoints::set_has_height() {
  _has_bits_[0] |= 0x00000020u;
}
inline void LidarPoints::clear_has_height() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void LidarPoints::clear_height() {
  height_ = 0u;
  clear_has_height();
}
inline ::google::protobuf::uint32 LidarPoints::height() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.height)
  return height_;
}
inline void LidarPoints::set_height(::google::protobuf::uint32 value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.height)
}

// optional uint32 width = 7;
inline bool LidarPoints::has_width() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void LidarPoints::set_has_width() {
  _has_bits_[0] |= 0x00000040u;
}
inline void LidarPoints::clear_has_width() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void LidarPoints::clear_width() {
  width_ = 0u;
  clear_has_width();
}
inline ::google::protobuf::uint32 LidarPoints::width() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.width)
  return width_;
}
inline void LidarPoints::set_width(::google::protobuf::uint32 value) {
  set_has_width();
  width_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.width)
}

// optional bool is_dense = 8;
inline bool LidarPoints::has_is_dense() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void LidarPoints::set_has_is_dense() {
  _has_bits_[0] |= 0x00000080u;
}
inline void LidarPoints::clear_has_is_dense() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void LidarPoints::clear_is_dense() {
  is_dense_ = false;
  clear_has_is_dense();
}
inline bool LidarPoints::is_dense() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.is_dense)
  return is_dense_;
}
inline void LidarPoints::set_is_dense(bool value) {
  set_has_is_dense();
  is_dense_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.is_dense)
}

// optional bool is_transform = 9;
inline bool LidarPoints::has_is_transform() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void LidarPoints::set_has_is_transform() {
  _has_bits_[0] |= 0x00000100u;
}
inline void LidarPoints::clear_has_is_transform() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void LidarPoints::clear_is_transform() {
  is_transform_ = false;
  clear_has_is_transform();
}
inline bool LidarPoints::is_transform() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.is_transform)
  return is_transform_;
}
inline void LidarPoints::set_is_transform(bool value) {
  set_has_is_transform();
  is_transform_ = value;
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.is_transform)
}

// optional string lidar_model = 10;
inline bool LidarPoints::has_lidar_model() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void LidarPoints::set_has_lidar_model() {
  _has_bits_[0] |= 0x00000200u;
}
inline void LidarPoints::clear_has_lidar_model() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void LidarPoints::clear_lidar_model() {
  if (lidar_model_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    lidar_model_->clear();
  }
  clear_has_lidar_model();
}
inline const ::std::string& LidarPoints::lidar_model() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.lidar_model)
  return *lidar_model_;
}
inline void LidarPoints::set_lidar_model(const ::std::string& value) {
  set_has_lidar_model();
  if (lidar_model_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    lidar_model_ = new ::std::string;
  }
  lidar_model_->assign(value);
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.lidar_model)
}
inline void LidarPoints::set_lidar_model(const char* value) {
  set_has_lidar_model();
  if (lidar_model_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    lidar_model_ = new ::std::string;
  }
  lidar_model_->assign(value);
  // @@protoc_insertion_point(field_set_char:Proto_msg.LidarPoints.lidar_model)
}
inline void LidarPoints::set_lidar_model(const char* value, size_t size) {
  set_has_lidar_model();
  if (lidar_model_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    lidar_model_ = new ::std::string;
  }
  lidar_model_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.LidarPoints.lidar_model)
}
inline ::std::string* LidarPoints::mutable_lidar_model() {
  set_has_lidar_model();
  if (lidar_model_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    lidar_model_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:Proto_msg.LidarPoints.lidar_model)
  return lidar_model_;
}
inline ::std::string* LidarPoints::release_lidar_model() {
  clear_has_lidar_model();
  if (lidar_model_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = lidar_model_;
    lidar_model_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void LidarPoints::set_allocated_lidar_model(::std::string* lidar_model) {
  if (lidar_model_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete lidar_model_;
  }
  if (lidar_model) {
    set_has_lidar_model();
    lidar_model_ = lidar_model;
  } else {
    clear_has_lidar_model();
    lidar_model_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.LidarPoints.lidar_model)
}

// optional string points_type = 11;
inline bool LidarPoints::has_points_type() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void LidarPoints::set_has_points_type() {
  _has_bits_[0] |= 0x00000400u;
}
inline void LidarPoints::clear_has_points_type() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void LidarPoints::clear_points_type() {
  if (points_type_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    points_type_->clear();
  }
  clear_has_points_type();
}
inline const ::std::string& LidarPoints::points_type() const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.points_type)
  return *points_type_;
}
inline void LidarPoints::set_points_type(const ::std::string& value) {
  set_has_points_type();
  if (points_type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    points_type_ = new ::std::string;
  }
  points_type_->assign(value);
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.points_type)
}
inline void LidarPoints::set_points_type(const char* value) {
  set_has_points_type();
  if (points_type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    points_type_ = new ::std::string;
  }
  points_type_->assign(value);
  // @@protoc_insertion_point(field_set_char:Proto_msg.LidarPoints.points_type)
}
inline void LidarPoints::set_points_type(const char* value, size_t size) {
  set_has_points_type();
  if (points_type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    points_type_ = new ::std::string;
  }
  points_type_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:Proto_msg.LidarPoints.points_type)
}
inline ::std::string* LidarPoints::mutable_points_type() {
  set_has_points_type();
  if (points_type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    points_type_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:Proto_msg.LidarPoints.points_type)
  return points_type_;
}
inline ::std::string* LidarPoints::release_points_type() {
  clear_has_points_type();
  if (points_type_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = points_type_;
    points_type_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void LidarPoints::set_allocated_points_type(::std::string* points_type) {
  if (points_type_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete points_type_;
  }
  if (points_type) {
    set_has_points_type();
    points_type_ = points_type;
  } else {
    clear_has_points_type();
    points_type_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:Proto_msg.LidarPoints.points_type)
}

// repeated float data = 12;
inline int LidarPoints::data_size() const {
  return data_.size();
}
inline void LidarPoints::clear_data() {
  data_.Clear();
}
inline float LidarPoints::data(int index) const {
  // @@protoc_insertion_point(field_get:Proto_msg.LidarPoints.data)
  return data_.Get(index);
}
inline void LidarPoints::set_data(int index, float value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:Proto_msg.LidarPoints.data)
}
inline void LidarPoints::add_data(float value) {
  data_.Add(value);
  // @@protoc_insertion_point(field_add:Proto_msg.LidarPoints.data)
}
inline const ::google::protobuf::RepeatedField< float >&
LidarPoints::data() const {
  // @@protoc_insertion_point(field_list:Proto_msg.LidarPoints.data)
  return data_;
}
inline ::google::protobuf::RepeatedField< float >*
LidarPoints::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:Proto_msg.LidarPoints.data)
  return &data_;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace Proto_msg

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Proto_5fmsg_2eLidarPoints_2eproto__INCLUDED
