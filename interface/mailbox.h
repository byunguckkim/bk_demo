
// Store data in the mailbox in your format. As data comes from the stack or
// your logs, store it here directly. For data that comes from Simian, convert
// it in the ConvertFromSimian function to your format and store it here, and
// data that's requested for Simian should be converted and returned from
// ConvertToSimian.

// For example, if you wanted to store and publish time as a ROS message,
// you would include the following:

// Delete this struct, it's just an example type.
struct Pose {
  int x, y;
};

// The mailbox struct should contain an instance of all of your
// custom messages in your format that need to be published to your stack
// or need to be converted to Simian format and sent to Simian.
struct Mailbox {
  Pose pose;

};