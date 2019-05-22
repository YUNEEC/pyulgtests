"""Tests that can be applied to ulg-files."""

from pyulgresample.ulogdataframe import DfUlg, TopicMsgs
from pyulgresample import loginfo
from pyulgresample import mathpandas as mpd
import pyulog
import os
import numpy as np
import pytest


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
        """Simple test for tilt."""
        dfulg = setup_dataframe(
            filepath, self._topics, self._zoh_topic_msgs_list
        )
        # During Manual / Stabilized and Altitude, the tilt threshold should not exceed
        # MPC_MAN_TILT_MAX
        tilt = mpd.tilt_from_attitude(
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
