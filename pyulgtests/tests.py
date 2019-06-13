"""Tests that can be applied to ulg-files."""

from pyulgresample.ulogdataframe import DfUlg, TopicMsgs
from pyulgresample import loginfo
from pyulgresample import mathpandas as mpd
import pyulog
import os
import numpy as np
import pytest
import matplotlib.pyplot as plt


def setup_dataframe(
    filepath, topics, zoh_topic_msgs=None, nan_topic_msgs=None
):
    """Setup dataframe for each test.

    This method has to be called for each new class.

    """
    try:
        dfulg = DfUlg.create(
            filepath,
            topics=topics,
            zoh_topic_msgs_list=zoh_topic_msgs,
            nan_topic_msgs_list=nan_topic_msgs,
        )
        return dfulg

    except Exception:
        pytest.skip("Could not create dfulg object")


class TestAttitude:
    """Tests for checking attitude."""

    @classmethod
    def setup_class(cls):
        """Set required topics for simple attitude analysis."""
        cls._topics = [
            "vehicle_attitude",
            "vehicle_attitude_setpoint",
            "vehicle_status",
        ]
        cls._zoh_topic_msgs_list = [TopicMsgs("vehicle_status", [])]

    def test_tilt_desired(self, filepath):
        """Simple test for tilt.

        During Manual / Stabilized and Altitude, the tilt threshold should not exceed MPC_MAN_TILT_MAX

        """
        dfulg = setup_dataframe(
            filepath, self._topics, self._zoh_topic_msgs_list
        )
        tilt = mpd.get_tilt_from_attitude(
            dfulg.df.T_vehicle_attitude_setpoint_0__F_q_d_0,
            dfulg.df.T_vehicle_attitude_setpoint_0__F_q_d_1,
            dfulg.df.T_vehicle_attitude_setpoint_0__F_q_d_2,
            dfulg.df.T_vehicle_attitude_setpoint_0__F_q_d_3,
            "T_vehicle_attitude_setpoint_0__F_z_xy",
        )

        man_tilt = (
            loginfo.get_param(dfulg.ulog, "MPC_MAN_TILT_MAX", 0) * np.pi / 180
        )
        assert dfulg.df[
            (
                (dfulg.df.T_vehicle_status_0__F_nav_state == 0)
                | (dfulg.df.T_vehicle_status_0__F_nav_state == 1)
            )
            & (tilt > man_tilt)
        ].empty


class TestYawModeGlobal:
    """Test for checking heading if global info is available."""

    @classmethod
    def setup_class(cls):
        """Set required topics for yaw analysis during auto-maneuver."""
        cls._topics = [
            "position_setpoint_triplet",
            "vehicle_local_position",
            "vehicle_local_position_setpoint",
            "home_position",
        ]

    def test_triplet_yaw_valid(self, filepath):
        """Simple test for triplet yaw-valid flag and triplet yaw.

        Whenever triplet yaw-flag is invalid, the triplet yaw has to be infinite.
        Whenever triplet yaw is infinite, the yaw-valid flag has to be False.

        """
        dfulg = setup_dataframe(filepath, self._topics)

        assert dfulg.df[
            (
                dfulg.df["T_position_setpoint_triplet_0__F_current_yaw_valid"]
                == 1
            )
            & ~(
                np.isfinite(
                    dfulg.df["T_position_setpoint_triplet_0__F_current_yaw"]
                )
            )
        ].empty

        assert dfulg.df[
            (
                ~np.isfinite(
                    dfulg.df["T_position_setpoint_triplet_0__F_current_yaw"]
                )
                & dfulg.df[
                    "T_position_setpoint_triplet_0__F_current_yaw_valid"
                ]
                == 1
            )
        ].empty

    # def test_mpc_yaw_mode(self, filepath):
    #    """Test wheter vehicle obeys mpc_yaw_mode during mission."""
    #    dfulg = setup_dataframe(
    #        filepath, self._topics
    #    )

    #    mpc_yaw_mode = {"towards_wp": 0, "towards_home": 1, "away_home": 2, "along_traj": 3}
    #    loginfo.add_param(dfulg, "MPC_YAW_MODE")

    #    plt.plot(dfulg.df.MPC_YAW_MODE)
    #    plt.plot(mpd.angle_wrap_pi(dfulg.df.T_vehicle_local_position_setpoint_0__F_yaw))
    #    plt.show()

    #    for g, d in dfulg.df.groupby(["MPC_YAW_MODE"]):

    #        if g == mpc_yaw_mode["away_home"]:

    #            df = d[d.T_position_setpoint_triplet_0__F_current_yaw_valid == False]

    #            north =  (df.T_home_position_0__F_x - d.T_vehicle_local_position_0__F_x) * -1
    #            east =   (df.T_home_position_0__F_y - d.T_vehicle_local_position_0__F_y) * -1
    #            heading = mpd.get_heading_from_2d_vector(north, east)
    #            #plt.plot(heading)
    #            plt.plot(mpd.angle_wrap_pi(df.T_vehicle_local_position_setpoint_0__F_yaw))
    #            #plt.plot(np.absolute(heading - df.T_vehicle_local_position_setpoint_0__F_yaw))
    #            plt.plot(d.MPC_YAW_MODE)
    #            plt.show()
    #            assert True
