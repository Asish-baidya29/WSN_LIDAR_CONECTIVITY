-- =============================================================================
-- wvs_landing_extension.sql
-- Additive migration for the drone-landing project.
-- Confirmed against wvs_1.sql: lidar_scan / lidar_point / lidar_object /
-- lidar_config do NOT exist in the current dump (they were seen empty in a
-- newer Workbench session, not in this schema file) — so they are created
-- fresh here, matching the existing table conventions (varchar(10) node_id,
-- InnoDB, FK to node_master, latin1 charset).
-- =============================================================================

-- ----------------------------
-- RSSI as a sensor type (Coordinator/Router side reporting — from earlier plan)
-- ----------------------------
INSERT INTO sensor_type_master (sensor_type_id, sensor_type_desc, sensor_uom)
VALUES ('RS', 'RSSI', 'dBm');

-- ----------------------------
-- Table: zone_status_log
-- One row per End-device zone check (inside/outside), from the Z<node_id> packet
-- ----------------------------
DROP TABLE IF EXISTS `zone_status_log`;
CREATE TABLE `zone_status_log` (
  `node_id`      varchar(10) NOT NULL,
  `zone_status`  tinyint(1) NOT NULL default '0',   -- 1 = inside, 0 = outside
  `receive_time` timestamp NOT NULL default CURRENT_TIMESTAMP,
  PRIMARY KEY (`node_id`, `receive_time`),
  CONSTRAINT `fk_zone_status_node_id` FOREIGN KEY (`node_id`)
    REFERENCES `node_master` (`node_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- ----------------------------
-- Table: lidar_scan
-- One row per sweep run (a "scan session"), from the drone's End device
-- ----------------------------
DROP TABLE IF EXISTS `lidar_scan`;
CREATE TABLE `lidar_scan` (
  `scan_id`      int NOT NULL AUTO_INCREMENT,
  `node_id`      varchar(10) NOT NULL,
  `start_time`   timestamp NOT NULL default CURRENT_TIMESTAMP,
  `end_time`     timestamp NULL,
  `zone_status`  tinyint(1) NOT NULL default '1',   -- should always be 1 (scan only happens inside zone)
  PRIMARY KEY (`scan_id`),
  KEY `fk_lidar_scan_node_id` (`node_id`),
  CONSTRAINT `fk_lidar_scan_node_id` FOREIGN KEY (`node_id`)
    REFERENCES `node_master` (`node_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- ----------------------------
-- Table: lidar_point
-- One row per raw servo-angle / distance sample within a scan, from L<node_id> packets
-- ----------------------------
DROP TABLE IF EXISTS `lidar_point`;
CREATE TABLE `lidar_point` (
  `scan_id`      int NOT NULL,
  `angle_deg`    smallint NOT NULL,
  `distance_cm`  smallint NOT NULL,
  `receive_time` timestamp NOT NULL default CURRENT_TIMESTAMP,
  PRIMARY KEY (`scan_id`, `angle_deg`, `receive_time`),
  CONSTRAINT `fk_lidar_point_scan_id` FOREIGN KEY (`scan_id`)
    REFERENCES `lidar_scan` (`scan_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- ----------------------------
-- Table: landing_decision_log
-- One row per chosen "best cell" result, from B<node_id> packets
-- ----------------------------
DROP TABLE IF EXISTS `landing_decision_log`;
CREATE TABLE `landing_decision_log` (
  `scan_id`         int NOT NULL,
  `node_id`         varchar(10) NOT NULL,
  `cell_x`          smallint NOT NULL,
  `cell_y`          smallint NOT NULL,
  `flatness_score`  smallint NOT NULL,   -- lower = flatter
  `receive_time`    timestamp NOT NULL default CURRENT_TIMESTAMP,
  PRIMARY KEY (`scan_id`),
  CONSTRAINT `fk_landing_decision_scan_id` FOREIGN KEY (`scan_id`)
    REFERENCES `lidar_scan` (`scan_id`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `fk_landing_decision_node_id` FOREIGN KEY (`node_id`)
    REFERENCES `node_master` (`node_id`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- ----------------------------
-- Register the drone's End-device node itself in node_master
-- (adjust node_id / node_type_id to match your actual addressing scheme —
--  check node_type_master for the correct type code for "End device")
-- ----------------------------
-- INSERT INTO node_master VALUES ('E50', 'E');   -- example only, confirm node_type_id
