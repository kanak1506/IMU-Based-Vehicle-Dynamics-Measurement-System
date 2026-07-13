import matplotlib
matplotlib.use('Agg')   # non-interactive backend; must come before any pyplot import

import streamlit as st
from dashboard.ui import render

st.set_page_config(
    page_title="IMU Vehicle Dynamics",
    layout="wide",
)

render()
