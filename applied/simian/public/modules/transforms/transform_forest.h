// Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/simian/public/utils/optional.h"

namespace applied {

// A transform forest is a collection of connected coordinate
// frames. Each link between frames has a source and a destination,
// and an associated transform encoded as a Pose3d object. The
// transform forest supports queries between any pair of frames, such
// that the caller does not need to worry about all the intervening
// links. It also supports generically iterating over all links, for
// instance to serialize and de-serialize the entire transform forest.
//
// Example: a truck has a radar on its chassis and a camera on its
// cab. One part of the sim determines the motion of the cab with
// respect to the chassis, and another part needs to create radar and
// camera data.
//
//    void mock_example_no_error_checking() {
//      TransformForest tff;
//      std::vector<Pose3d> poses;
//      read_scenario_poses(&poses);
//      tff.SetOrAdd("world", "ego", poses[0]);
//      tff.SetOrAdd("ego", "chassis", poses[1]);
//      tff.SetOrAdd("chassis", "cab", poses[2]);
//      tff.SetOrAdd("chassis", "radar", poses[3]);
//      tff.SetOrAdd("cab", "camera", poses[4]);
//      while (run_sim()) {
//        // sensor sim
//        render_camera(tff.Query("world", "camera"));
//        render_radar(tff.Query("world", "radar"));
//
//        // vehicle sim
//        tff.SetOrAdd("world", "ego", move_ego());
//        tff.SetOrAdd("chassis", "cab", get_cab_sway());
//      }
//    }
//
class TransformForest {
 public:
  // Function signature for visiting each link in the forest: you
  // receive the source and destination names, and the Pose3d that
  // encodes how things that are expressed in destination frame
  // coordinates can be changed to source frame coordinates.
  using VisitCallback =
      std::function<void(const std::string& src, const std::string& dst, const Pose3d& src_T_dst)>;

  // Look up or compute the transformation between the given source
  // and destination. The src and dst given here need not be directly
  // related in the forest: the forest is traversed to find the
  // connected path between the two and compose the transforms of the
  // links along that path.
  //
  // Note that this can fail if the two frames are not connected. In
  // that case, std::nullopt is returned. Likewise if one or both of
  // the frames are not in the forest at all.
  nonstd::optional<Pose3d> Query(const std::string& src, const std::string& dst) const;

  // Iterates over all links in the forest and calls the given
  // callback for each of them. The iteration order is implementation
  // defined, avoid depending on it.
  void Visit(VisitCallback callback) const;

  // Visit only the links that have been updated (via SetOrAdd(), AddAndSet(), or Set())
  // since the most recent call to IncrementVersion() (or since the forest was constructed, if
  // IncrementVersion() has not yet been called).
  void VisitDiff(VisitCallback callback) const;

  // Convenience method for printing the entire forest to a
  // human-reabable debug format.
  const std::string DebugString() const;

  // Sets the transform of the link from a given source to a given
  // destination frame. If that link is not yet in the forest, add it
  // first.
  //
  // Note that this can fail if adding the src-dst link would create a
  // loop. In that case, the forest is not modified, and this method
  // returns false. For this function, adding B->A when A->B already exists
  // is not a "loop"; rather, the transform between A and B is set appropriately.
  bool SetOrAdd(const std::string& src, const std::string& dst, const Pose3d& src_T_dst);

  // This is just like SetOrAdd, except that will not add links that already exist (in
  // either direction).  That is, if A->B exists, then calling this function to add
  // either A->B or B->A will return false.
  bool AddAndSet(const std::string& src, const std::string& dst, const Pose3d& src_T_dst);

  // Set the transform for an existing link; the function will infer the parent from `dst`.
  // Returns false if no frame `dst` exists or if `dst` does not have a parent frame.
  bool Set(const std::string& dst, const Pose3d& src_T_dst);

  // This just allows checking whether or not the transform forest includes a certain frame.
  bool HasFrame(const std::string& name) const;

  // Obtain the name of the parent frame to the frame named `name`.
  // Returns nullopt if the transform forest does not include a frame named `name` or if
  // the frame named `name` has no parent.
  nonstd::optional<std::string> GetParentFrameName(const std::string& name) const;

  // Returns the names of all frames in the connected components of the given frame, including
  // the given frame itself, provided it's in the transform forest.  Otherwise returns empty.
  std::vector<std::string> GetConnectedNodes(const std::string& name) const;

  // Returns a map of all names of connected frames to the transform from the given frame, including
  // the given frame itself, provided it's in the transform forest.  Otherwise returns empty.
  std::map<std::string, Pose3d> GetConnectedTransforms(const std::string& name) const;

  // Returns all frame names
  std::vector<std::string> GetAllFrameNames() const;

  // Use this function together with VisitDiff().
  void IncrementVersion();

 private:
  // Driver for SetOrAdd and AddAndSet.
  bool SetOrAddImpl(const std::string& src, const std::string& dst, const Pose3d& src_T_dst,
                    bool add_only);

  // Implementation notes:
  //
  // The trees are maintained as "Frame" instances, each having a
  // pointer to its parent and a set of pointers to its children. The
  // core algorithms only really needs the parent pointers, the
  // children are tracked mainly to ease maintening the depth fields,
  // which in turn makes the search for common ancestors much more
  // efficient. The sets of children are kept in alphabetically sorted
  // order, which is not essential but really eases testing and
  // debugging.
  //
  // The pointers in the trees are all raw. All the Frame instances
  // are owned by the TransformForest, simply by storing them as
  // unique_ptrs (in a hash map that also serves as an efficient
  // name-based lookup). That greatly simplifies the helper methods
  // that deal with Frame instances and the trees they are part of.

  struct Frame;
  struct CompareFrameNames {
    bool operator()(const Frame* lhs, const Frame* rhs) const { return lhs->name < rhs->name; }
  };

  struct Frame {
    explicit Frame(const std::string& _name)
        : name(_name), parent_T_this(), parent(nullptr), children(), depth(0) {}

    void SetTransform(const Pose3d& parent_T_this_arg, const int version) {
      parent_T_this = parent_T_this_arg;
      most_recently_updated_in_version = version;
    }

    const std::string name;

    // Note that currently we end up calling .Inverse() quite
    // frequently. It probably makes sense to track both directions of
    // this transforms, and also just swap the two whenever we want to
    // replace the transform with its own inverse. Let's first see
    // though what the runtime hit is in practice.
    Pose3d parent_T_this;

    Frame* parent;
    std::set<Frame*, CompareFrameNames> children;
    int depth;
    int most_recently_updated_in_version = 0;
  };

  static void SetDepthRecurse(Frame* frame, int new_depth);

  static void DumpRecurse(std::ostream& os, const std::string& prefix, const Frame* root);

  static void VisitRecurse(const Frame* root, VisitCallback callback);

  static void VisitDiffRecurse(const std::int32_t current_version, const Frame* root,
                               VisitCallback callback);

  Frame* CreateFrame(const std::string& name);
  Frame* FindFrame(const std::string& name) const;
  void SetAncestry(Frame* parent, Frame* child);
  void PickUpRecurse(Frame* new_parent, Frame* frame, const applied::Pose3d& new_parent_T_this);
  Frame* MergeTrees(Frame* aa, Frame* bb, const applied::Pose3d& a_T_b);

  std::unordered_map<std::string, std::unique_ptr<Frame>> frames_;
  std::set<const Frame*, CompareFrameNames> roots_;

  // Versioning of the transform forest permits visiting only the links that have been updated
  // after a checkpoint (by calling IncrementVersion()).
  int current_version_ = 0;
  bool modified_since_increment_version_ = false;
};

}  // namespace applied
