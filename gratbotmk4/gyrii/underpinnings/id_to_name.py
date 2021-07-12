#
import uuid

def id_to_name(id):
    id_names_hash=["Franz","Brunhilda","Steve","Persepolis","Legolas","Senator","Doublewide","Convolution","Beaucephalus","Microencephalus","Enchilada","Buttercup","Malferious","Ferrous","Titiana","Europa","Malpractice","Daedelus","Dad-elus","Cheesemonger","Burgertime","ForgetMeNot","Nevar4get","Allowance","Betrayal","Elvis"]
    return id_names_hash[hash(id)%len(id_names_hash)]
