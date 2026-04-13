import { benchmark } from './index';
import { runModified, runOriginal } from './functions';

benchmark('original', runOriginal);
benchmark('modified', runModified);
