#include <ecrt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <memory>

static ec_master_t * master = nullptr;
static ec_domain_t * domain = nullptr;
static uint8_t * domain_pd = nullptr;
static volatile bool running = true;

static unsigned int off_fx = 0;
static unsigned int off_fy = 0;
static unsigned int off_fz = 0;
static unsigned int off_mx = 0;
static unsigned int off_my = 0;
static unsigned int off_mz = 0;
static unsigned int off_status = 0;
static unsigned int off_sample = 0;
static unsigned int off_temp = 0;

struct SensorSharedData {
  int32_t fx;
  int32_t fy;
  int32_t fz;
  int32_t mx;
  int32_t my;
  int32_t mz;
  int32_t status;
  int32_t sample;
  int32_t temper;
};

static std::shared_ptr<boost::interprocess::shared_memory_object> shm_ptr;
static std::shared_ptr<boost::interprocess::mapped_region> mapped_ptr;
static SensorSharedData * shm_data = nullptr;

static void signal_handler(int /*sig*/) {
  running = false;
}

static void init_shared_memory() {
  boost::interprocess::permissions perm;
  perm.set_unrestricted();
  boost::interprocess::shared_memory_object::remove("Ethercat_sensor_real");
  shm_ptr = std::make_shared<boost::interprocess::shared_memory_object>(
      boost::interprocess::open_or_create,
      "Ethercat_sensor_real",
      boost::interprocess::read_write,
      perm);
  shm_ptr->truncate(sizeof(SensorSharedData));
  mapped_ptr = std::make_shared<boost::interprocess::mapped_region>(*shm_ptr, boost::interprocess::read_write);
  std::memset(mapped_ptr->get_address(), 0, mapped_ptr->get_size());
  shm_data = static_cast<SensorSharedData *>(mapped_ptr->get_address());
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  init_shared_memory();

  const uint16_t alias = 0;
  const uint16_t position = 0;
  const uint32_t vendor_id = 0x000000a1;
  const uint32_t product_code = 0x00008081;

  master = ecrt_request_master(2);
  if (!master) {
    fprintf(stderr, "Failed to request master2.\n");
    return 1;
  }

  domain = ecrt_master_create_domain(master);
  if (!domain) {
    fprintf(stderr, "Failed to create domain.\n");
    return 1;
  }

  ec_slave_config_t * sc = ecrt_master_slave_config(master, alias, position, vendor_id, product_code);
  if (!sc) {
    fprintf(stderr, "Failed to get slave config.\n");
    return 1;
  }

  ec_pdo_entry_info_t slave_0_pdo_entries[] = {
      {0x2000, 0x00, 32}, /* ControlCode */
      {0x2001, 0x00, 32}, /* x */
      {0x2002, 0x00, 32}, /* y */
      {0x2003, 0x00, 32}, /* z */
      {0x2004, 0x00, 32}, /* a */
      {0x2005, 0x00, 32}, /* b */
      {0x2006, 0x00, 32}, /* c */
      {0x2007, 0x00, 32}, /* d */
      {0x4000, 0x00, 32}, /* Fx */
      {0x4001, 0x00, 32}, /* Fy */
      {0x4002, 0x00, 32}, /* Fz */
      {0x4003, 0x00, 32}, /* Mx */
      {0x4004, 0x00, 32}, /* My */
      {0x4005, 0x00, 32}, /* Mz */
      {0x4006, 0x00, 32}, /* StatusCode */
      {0x4007, 0x00, 32}, /* SampleCounter */
      {0x4008, 0x00, 32}, /* Temper */
  };

  ec_pdo_info_t slave_0_pdos[] = {
      {0x1600, 8, slave_0_pdo_entries + 0}, /* RxPDO 1 */
      {0x1a00, 9, slave_0_pdo_entries + 8}, /* TxPDO 1 */
  };

  ec_sync_info_t slave_0_syncs[] = {
      {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
      {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
      {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
      {0xff}
  };

  if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return 1;
  }

  ec_pdo_entry_reg_t domain_regs[] = {
      {alias, position, vendor_id, product_code, 0x4000, 0x00, &off_fx},
      {alias, position, vendor_id, product_code, 0x4001, 0x00, &off_fy},
      {alias, position, vendor_id, product_code, 0x4002, 0x00, &off_fz},
      {alias, position, vendor_id, product_code, 0x4003, 0x00, &off_mx},
      {alias, position, vendor_id, product_code, 0x4004, 0x00, &off_my},
      {alias, position, vendor_id, product_code, 0x4005, 0x00, &off_mz},
      {alias, position, vendor_id, product_code, 0x4006, 0x00, &off_status},
      {alias, position, vendor_id, product_code, 0x4007, 0x00, &off_sample},
      {alias, position, vendor_id, product_code, 0x4008, 0x00, &off_temp},
      {}
  };

  if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
    fprintf(stderr, "Failed to register PDO entries.\n");
    return 1;
  }

  if (ecrt_master_activate(master)) {
    fprintf(stderr, "Failed to activate master.\n");
    return 1;
  }

  domain_pd = ecrt_domain_data(domain);
  if (!domain_pd) {
    fprintf(stderr, "Failed to get domain data.\n");
    return 1;
  }

  struct timespec sleep_time = {0, 1000000}; // 1ms

  while (running) {
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    shm_data->fx = EC_READ_S32(domain_pd + off_fx);
    shm_data->fy = EC_READ_S32(domain_pd + off_fy);
    shm_data->fz = EC_READ_S32(domain_pd + off_fz);
    shm_data->mx = EC_READ_S32(domain_pd + off_mx);
    shm_data->my = EC_READ_S32(domain_pd + off_my);
    shm_data->mz = EC_READ_S32(domain_pd + off_mz);
    shm_data->status = EC_READ_S32(domain_pd + off_status);
    shm_data->sample = EC_READ_S32(domain_pd + off_sample);
    shm_data->temper = EC_READ_S32(domain_pd + off_temp);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    nanosleep(&sleep_time, nullptr);
  }

  ecrt_release_master(master);
  return 0;
}
