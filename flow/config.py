"""Default config variables, which may be overridden by a user config."""
import os.path as osp
import os

PYTHON_COMMAND = "python"

SUMO_SLEEP = 1.0  # Delay between initializing SUMO and connecting with TraCI

PROJECT_PATH = osp.abspath(osp.join(osp.dirname(__file__), '..'))

LOG_DIR = PROJECT_PATH + "/data"

# users set both of these in their bash_rc or bash_profile
# and also should run aws configure after installing awscli
AWS_ACCESS_KEY = os.environ.get("AWS_ACCESS_KEY", None)

AWS_ACCESS_SECRET = os.environ.get("AWS_ACCESS_SECRET", None)

AWS_S3_PATH = "s3://bucket_name"


# ===========================================================================
# =========================== Aimsun config  ================================

# path to the Aimsun_Next main directory (required for Aimsun simulations)
AIMSUN_NEXT_PATH = os.environ.get("AIMSUN_NEXT_PATH", None)


# Constants for the TCP connection to Aimsun
HOST = '127.0.0.1' #localhost
# The PORT is now a random integer 1024 <= PORT < 32768
STATRESP = b'1'
STATRESP_LEN = 1

# Client identifiers
NETWORK_LOAD_ID   = b'load.py_and_generate.py'
RUN_API_ID        = b'run.py_and_api.py'

# An attempt to create a socket between Wolf and Aimsun
# will time out after the following duration (in seconds)
TIMEOUT_T = 120

VERSION = 'Aimsun integration'

WOLF_SEED = 3526