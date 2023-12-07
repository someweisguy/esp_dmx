#include "include/queue_status.h"

#include "dmx/include/driver.h"
#include "dmx/include/struct.h"
#include "rdm/utils/bus_ctl.h"

// static int rdm_rhd_status_messages(dmx_port_t dmx_num, rdm_header_t *header,
//                                    void *pd, uint8_t *pdl_out,
//                                    const rdm_pd_schema_t *schema) {
//   *pdl_out = 0;  // TODO: implement status messages
//   return RDM_RESPONSE_TYPE_ACK;
// }

// static int rdm_rhd_queued_message(dmx_port_t dmx_num, rdm_header_t *header,
//                                   void *pd, uint8_t *pdl_out,
//                                   const rdm_pd_schema_t *schema) {
//   // Verify data is valid
//   const uint8_t status_type_requested = *(uint8_t *)pd;
//   if (status_type_requested != RDM_STATUS_GET_LAST_MESSAGE &&
//       status_type_requested != RDM_STATUS_ADVISORY &&
//       status_type_requested != RDM_STATUS_WARNING &&
//       status_type_requested != RDM_STATUS_ERROR) {
//     *pdl_out = rdm_pd_serialize_word(pd, RDM_NR_DATA_OUT_OF_RANGE);
//     return RDM_RESPONSE_TYPE_NACK_REASON;
//   }  // TODO: ensure error-checking is correct

//   int ack;

//   // Pop a PID from the queue and attempt to serve the queued data
//   const rdm_pid_t queue_pid = rdm_queue_pop(dmx_num);
//   if (queue_pid != 0) {
//     // Call the response handler for the queued PID
//     header->pid = queue_pid;
//     ack = rdm_pd_call_response_handler(dmx_num, header, pd, pdl_out);
//   } else {
//     // When there aren't any queued messages respond with a status message
//     header->pid = RDM_PID_STATUS_MESSAGE;
//     const rdm_pd_schema_t *new_schema =
//         rdm_pd_get_schema(dmx_num, header->pid, header->sub_device);
//     ack = rdm_rhd_status_messages(dmx_num, header, pd, pdl_out, new_schema);
//   }

//   return ack;
// }

bool rdm_register_queued_message(dmx_port_t dmx_num, rdm_callback_t cb,
                                 void *context) {
  DMX_CHECK(dmx_num < DMX_NUM_MAX, false, "dmx_num error");
  DMX_CHECK(dmx_driver_is_installed(dmx_num), false, "driver is not installed");

  // // Define the parameter
  // const rdm_pid_t pid = RDM_PID_QUEUED_MESSAGE;
  // const rdm_pd_definition_t def = {
  //     .schema = {.data_type = RDM_DS_NOT_DEFINED,
  //                .cc = RDM_CC_GET,
  //                .pdl_size = sizeof(uint8_t),
  //                .alloc_size = 0,  // Parameter is deterministic
  //                .format = "b$"},
  //     .nvs = false,
  //     .response_handler = rdm_rhd_queued_message,
  // };

  // rdm_pd_add_deterministic(dmx_num, pid, RDM_SUB_DEVICE_ROOT, &def, NULL);
  // return rdm_pd_update_callback(dmx_num, pid, RDM_SUB_DEVICE_ROOT, cb, context);

  return false;
}