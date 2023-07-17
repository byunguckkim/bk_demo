// Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/simian/public/modules/transforms/transform_forest.h"

#include <queue>
#include <sstream>

#include "applied/simian/public/modules/transforms/spatial.h"

namespace applied {

bool TransformForest::AddAndSet(const std::string& src, const std::string& dst,
                                const Pose3d& src_T_dst) {
  Frame* fsrc = FindFrame(src);
  Frame* fdst = FindFrame(dst);
  if ((fsrc && fdst) && (fsrc->parent == fdst || fdst->parent == fsrc)) {
    return false;
  }
  return SetOrAdd(src, dst, src_T_dst);
}

bool TransformForest::SetOrAdd(const std::string& src, const std::string& dst,
                               const Pose3d& src_T_dst) {
  if (src == dst) {
    return false;
  }

  // There's a few cases below where we don't actually modify the forest
  // but we are conservative here by considering all cases below to modify the forest
  // (that is, VisitDiff() will report a link that hasn't actually been modified).
  modified_since_increment_version_ = true;

  Frame* fsrc = FindFrame(src);
  Frame* fdst = FindFrame(dst);

  // Add one or both.
  if (!fsrc || !fdst) {
    if (!fsrc) {
      fsrc = CreateFrame(src);
    }
    if (!fdst) {
      fdst = CreateFrame(dst);
    }
    if (fdst->parent) {
      SetAncestry(fdst, fsrc);
      SetDepthRecurse(fsrc, fdst->depth + 1);
      fsrc->SetTransform(src_T_dst.Inverse(), current_version_);
      return true;
    }
    SetAncestry(fsrc, fdst);
    SetDepthRecurse(fdst, fsrc->depth + 1);
    fdst->SetTransform(src_T_dst, current_version_);
    return true;
  }

  // Easy: existing link
  if (fdst->parent == fsrc) {
    fdst->SetTransform(src_T_dst, current_version_);
    return true;
  }

  // Also easy: reverse of existing link
  if (fsrc->parent == fdst) {
    fsrc->SetTransform(src_T_dst.Inverse(), current_version_);
    return true;
  }

  // We need to "pick up" one of the trees (partially reverse its
  // parent/child relationships) and graft it onto the other, but this
  // only works if they weren't connected in the first place. If
  // MergeTrees() returns a nullptr, it failed (fsrc and fdst were in
  // the same tree already).
  return MergeTrees(fsrc, fdst, src_T_dst);
}

bool TransformForest::Set(const std::string& dst, const Pose3d& src_T_dst) {
  const Frame* fdst = FindFrame(dst);
  if (!fdst || !fdst->parent) {
    return false;
  }
  return SetOrAdd(fdst->parent->name, dst, src_T_dst);
}

void TransformForest::Visit(VisitCallback callback) const {
  for (const auto& root : roots_) {
    VisitRecurse(root, callback);
  }
}

void TransformForest::VisitDiff(VisitCallback callback) const {
  if (!modified_since_increment_version_) {
    // Avoid recursion if there's been no modifications.
    return;
  }
  for (const auto& root : roots_) {
    VisitDiffRecurse(current_version_, root, callback);
  }
}

const std::string TransformForest::DebugString() const {
  std::ostringstream oss;
  std::string prefix = "";
  for (const auto& root : roots_) {
    DumpRecurse(oss, prefix, root);
  }

  return oss.str();
}

nonstd::optional<Pose3d> TransformForest::Query(const std::string& src,
                                                const std::string& dst) const {
  const Frame* fsrc = FindFrame(src);
  if (!fsrc) {
    return nonstd::nullopt;
  }
  const Frame* fdst = FindFrame(dst);
  if (!fdst) {
    return nonstd::nullopt;
  }

  // Walk to the root from both src and dst. We're done if we hit a
  // common non-null frame. Otherwise, src and dst are not on the same
  // tree.
  //
  // This approach makes sense given that the (vast) majority of
  // queries can be expected to be valid.
  Pose3d common_T_src, common_T_dst;
  while (fsrc->depth > fdst->depth) {
    common_T_src = fsrc->parent_T_this * common_T_src;
    fsrc = fsrc->parent;
  }
  while (fdst->depth > fsrc->depth) {
    common_T_dst = fdst->parent_T_this * common_T_dst;
    fdst = fdst->parent;
  }
  while (fsrc != fdst) {
    // Here we know that fsrc and fdst are at the same depth and can
    // be walked up the tree in lockstep. What may happen though is
    // that fsrc and fdst are both roots (of separate trees) in which
    // case they will be different but both will have no parent.
    if (!fsrc->parent) {
      return nonstd::nullopt;
    }
    common_T_src = fsrc->parent_T_this * common_T_src;
    common_T_dst = fdst->parent_T_this * common_T_dst;
    fsrc = fsrc->parent;
    fdst = fdst->parent;
  }

  return common_T_src.Inverse() * common_T_dst;
}

void TransformForest::SetDepthRecurse(Frame* frame, int new_depth) {
  frame->depth = new_depth;
  ++new_depth;
  for (auto* child : frame->children) {
    SetDepthRecurse(child, new_depth);
  }
}

void TransformForest::DumpRecurse(std::ostream& os, const std::string& prefix, const Frame* root) {
  os << prefix << root->name << " d " << root->depth << " t "
     << root->parent_T_this.Translation().transpose() << "\n";
  for (const auto& child : root->children) {
    DumpRecurse(os, prefix + "  ", child);
  }
}

void TransformForest::SetAncestry(Frame* parent, Frame* child) {
  Frame* prev_parent = child->parent;
  // Maintain children sets.
  if (prev_parent) {
    prev_parent->children.erase(child);
  }
  child->parent = parent;
  if (parent) {
    parent->children.emplace(child);
  }
  // Maintain root set.
  if (!prev_parent && parent) {
    roots_.erase(child);
  } else if (prev_parent && !parent) {
    roots_.emplace(child);
  }
}

void TransformForest::PickUpRecurse(Frame* new_parent, Frame* frame,
                                    const applied::Pose3d& new_parent_T_this) {
  if (!frame) {
    return;
  }
  PickUpRecurse(frame, frame->parent, frame->parent_T_this.Inverse());
  SetAncestry(new_parent, frame);
  // Note: this recursion updates the version for frames that are swapped as a side
  // effect of another link being modified, leading to "false positives" when
  // invoking VisitDiff().
  frame->SetTransform(new_parent_T_this, this->current_version_);
}

// Returns aa or bb to designate which one became a child of the
// other. Returns nullptr if the trees could not be merged (i.e. aa
// and bb were already in the same tree).
TransformForest::Frame* TransformForest::MergeTrees(Frame* aa, Frame* bb,
                                                    const applied::Pose3d& a_T_b) {
  {  // Cannot merge if aa and bb are on the same tree.
    const Frame* ia = aa;
    const Frame* ib = bb;
    while (ia->depth > ib->depth) {
      ia = ia->parent;
    }
    while (ib->depth > ia->depth) {
      ib = ib->parent;
    }
    while (ia != ib) {
      ia = ia->parent;
      ib = ib->parent;
    }
    if (ia) {  // We found a common ancestor.
      return nullptr;
    }
  }

  // Heuristic about which frame to "pick up" and graft as new subtree
  // onto the other: the one at lesser current depth will have fewer
  // child-parent swaps during PickUpRecurse(). So that's probably
  // better. Keep in mind that the one being picked up is the /second/
  // argument to PickUpRecurse().
  if (aa->depth < bb->depth) {
    PickUpRecurse(bb, aa, a_T_b.Inverse());
    SetDepthRecurse(aa, bb->depth + 1);
    return aa;
  }
  PickUpRecurse(aa, bb, a_T_b);
  SetDepthRecurse(bb, aa->depth + 1);
  return bb;
}

void TransformForest::VisitRecurse(const Frame* root, VisitCallback callback) {
  for (const auto* child : root->children) {
    callback(root->name, child->name, child->parent_T_this);
    VisitRecurse(child, callback);
  }
}

void TransformForest::VisitDiffRecurse(const int current_version, const Frame* root,
                                       VisitCallback callback) {
  for (const auto* child : root->children) {
    if (child->most_recently_updated_in_version >= current_version) {
      callback(root->name, child->name, child->parent_T_this);
    }
    VisitDiffRecurse(current_version, child, callback);
  }
}

TransformForest::Frame* TransformForest::CreateFrame(const std::string& name) {
  // Should never happen, so might be best to avoid the runtime cost
  // of checking all the time. However, if we do hit this, it is
  // better to segfault on the returned null, rather than keep running
  // but with (1) a memory leak due to how emplace handles duplicates
  // and (2) no obvious error other than maybe someone will wonder why
  // they are getting unexpected transforms (some of the time) from
  // Query().
  if (0 != frames_.count(name)) {
    return nullptr;
  }

  Frame* frame = new Frame(name);
  frames_.emplace(name, frame);
  roots_.emplace(frame);
  return frame;
}

bool TransformForest::HasFrame(const std::string& name) const {
  auto ii = frames_.find(name);
  return (frames_.end() != ii);
}

nonstd::optional<std::string> TransformForest::GetParentFrameName(const std::string& name) const {
  const Frame* frame = FindFrame(name);
  if (!frame || !frame->parent) {
    return nonstd::nullopt;
  }
  return frame->parent->name;
}

TransformForest::Frame* TransformForest::FindFrame(const std::string& name) const {
  auto ii = frames_.find(name);
  if (frames_.end() == ii) {
    return nullptr;
  }
  return ii->second.get();
}

// Returns the names of all frames in the connected component of the given frame, including
// the given frame itself, provided it's in the transform forest.  Otherwise returns empty.
std::vector<std::string> TransformForest::GetConnectedNodes(const std::string& name) const {
  std::vector<std::string> retval;
  Frame* frame = FindFrame(name);
  if (!frame) {
    return retval;  // empty vector
  }
  while (frame->parent) {
    frame = frame->parent;
  }
  // Now `frame` is the root of the connected component.
  retval.push_back(frame->name);
  VisitCallback cb = [&retval](const std::string& src, const std::string& dst,
                               const applied::Pose3d& src_T_dst) { retval.push_back(dst); };
  VisitRecurse(frame, cb);
  return retval;
}

std::map<std::string, Pose3d> TransformForest::GetConnectedTransforms(
    const std::string& name) const {
  std::map<std::string, Pose3d> retval;
  Frame* frame = FindFrame(name);
  if (!frame) {
    return retval;  // empty map
  }
  Pose3d pose;
  while (frame->parent) {
    pose = pose * frame->parent_T_this.Inverse();
    frame = frame->parent;
  }
  // Now `frame` is the root of the connected component.  Do BFS.
  std::queue<std::pair<const Frame*, Pose3d>> fifo;
  fifo.push({frame, pose});
  while (!fifo.empty()) {
    const Frame* current;
    Pose3d current_pose;
    std::tie(current, current_pose) = fifo.front();
    fifo.pop();
    retval.insert({current->name, current_pose});
    for (const Frame* child : current->children) {
      fifo.push({child, current_pose * child->parent_T_this});
    }
  }
  return retval;
}

std::vector<std::string> TransformForest::GetAllFrameNames() const {
  std::vector<std::string> retval;
  retval.reserve(frames_.size());
  for (const auto& ff : frames_) {
    retval.push_back(ff.first);
  }
  return retval;
}

void TransformForest::IncrementVersion() {
  modified_since_increment_version_ = false;
  ++current_version_;
}

}  // namespace applied
